package lib.ironpulse.limelight;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.RobotState;
import java.util.Arrays;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import lib.ironpulse.utils.LimelightHelpers;

/**
 * Limelight IO. Takes in a LimelightConfig and odometry from swerve for enhanced detection. Should
 * be instantiated for every limelight.
 */
public class LimelightIOReal implements LimelightIO {
    public final LimelightIOConfig config;
    private final DoubleSupplier yawSupplier;
    private final BooleanSupplier rejectionSupplier;
    private boolean isPrevDisabled = true;

    public LimelightIOReal(
            LimelightIOConfig config,
            DoubleSupplier yawSupplier,
            BooleanSupplier rejectionSupplier) {
        this.config = config;
        this.yawSupplier = yawSupplier;
        this.rejectionSupplier = rejectionSupplier;
        if (config.useInternalIMU && !config.isLimelight4) {
            throw new IllegalArgumentException("Internal IMU only exists on limelight 4");
        } else if (config.useInternalIMU) {
            LimelightHelpers.SetIMUAssistAlpha(config.name, config.filterAlpha);
            setIMUMode();
        }
    }

    @Override
    public boolean canUseInternalIMU() {
        return config.mountPosition == LimelightIOConfig.MountPosition.ON_ROBOT
                && config.useInternalIMU;
    }

    /**
     * The reliability score ranges from 0 to 1 (inclusive) and indicates the trustworthiness of
     * this limelight's prediction accounting its prediction weight.
     *
     * @return 0-1 (inclusive). 1 is the most reliable and 0 the least.
     */
    @Override
    public double getReliabilityScore(LimelightHelpers.PoseEstimate poseEstimate) {
        if (poseEstimate.tagCount == 0 || rejectionSupplier.getAsBoolean()) {
            // no tags or decided to reject
            return 0;
        } else if (poseEstimate.tagCount == 1) {
            return 0.5 * config.weight;
        } else {
            // tags >= 2
            return 1 * config.weight;
        }
    }

    @Override
    public String getName() {
        return config.name;
    }

    /**
     * Switches the internal IMU between states.
     *
     * <p>Context: called upon initialization and change of robot states (Disabled <-> Enabled)
     */
    private void setIMUMode() {
        if (!canUseInternalIMU()) {
            // I don't really think internal IMU would be stable on limelights
            // mounted on mechanisms (e.g. turrets, shooters) as the yaw reading
            // would be different from that of the robot.
            // So we do not trust the internal IMU completely...
            LimelightHelpers.SetIMUMode(config.name, InternalIMUMode.EXTERNAL_ONLY.getValue());
            return;
        }
        if (RobotState.isDisabled()) {
            // disabled; use IMU mode 1 - seed internal IMU with data
            LimelightHelpers.SetIMUMode(config.name, InternalIMUMode.EXTERNAL_SEED.getValue());
        } else {
            // enabled - use IMU mode 4 - externally assisted internal IMU MegaTag2
            LimelightHelpers.SetIMUMode(
                    config.name, InternalIMUMode.INTERNAL_EXTERNAL_ASSIST.getValue());
        }
    }

    @Override
    public void setPipeline(int pipeline) {
        LimelightHelpers.setPipelineIndex(config.name, pipeline);
    }

    @Override
    public void setLEDMode(LEDMode mode) {
        switch (mode) {
            case ON -> LimelightHelpers.setLEDMode_ForceOn(config.name);
            case OFF -> LimelightHelpers.setLEDMode_ForceOff(config.name);
            case BLINK -> LimelightHelpers.setLEDMode_ForceBlink(config.name);
            case PIPELINE_CONTROL -> LimelightHelpers.setLEDMode_PipelineControl(config.name);
        }
    }

    @Override
    public int getPipeline() {
        return (int) LimelightHelpers.getCurrentPipelineIndex(config.name);
    }

    @Override
    public void setAprilTagIdFilter(int[] ids) {
        LimelightHelpers.SetFiducialIDFiltersOverride(config.name, ids);
    }

    @Override
    public void updateInputs(LimelightIOInputs inputs) {
        if (canUseInternalIMU() && isPrevDisabled != RobotState.isDisabled()) {
            setIMUMode();
        }
        isPrevDisabled = RobotState.isDisabled();
        LimelightHelpers.SetRobotOrientation(
                config.name,
                yawSupplier.getAsDouble(),
                0,
                0,
                0,
                0,
                0); // the last 5 parameters are not necessary

        // generate pose Estimate
        LimelightHelpers.PoseEstimate estimate;
        if (config.useMegaTag2) {
            estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(config.name);
        } else {
            estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(config.name);
        }
        // FIXME: need testing - two versions of inputs.pose
        //        inputs.pose = new Pose3d(estimate.pose).rotateBy(
        //                new Rotation3d(0, 0, Radians.convertFrom(getIMUYawRobot(), Degrees)));
        inputs.pose = new Pose3d(estimate.pose);
        inputs.timestampSeconds = estimate.timestampSeconds;
        inputs.latency = estimate.latency;
        inputs.detectedTagIds = Arrays.stream(estimate.rawFiducials).mapToInt(r -> r.id).toArray();
        inputs.reliability = getReliabilityScore(estimate);
    }

    @Override
    public double getIMUYawInternal() {
        return LimelightHelpers.getIMUData(config.name).Yaw;
    }

    @Override
    public double getIMUYawRobot() {
        return LimelightHelpers.getIMUData(config.name).robotYaw;
    }
}
