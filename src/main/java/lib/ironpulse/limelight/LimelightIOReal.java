package lib.ironpulse.limelight;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.RobotState;
import java.util.Arrays;
import java.util.function.DoubleSupplier;
import lib.ironpulse.utils.LimelightHelpers;

/**
 * Limelight IO. Takes in a LimelightConfig and odometry from swerve for enhanced detection. Should
 * be instantiated for every limelight.
 */
public class LimelightIOReal implements LimelightIO {
    public final LimelightIOConfig config;
    private final DoubleSupplier yawSupplier;

    public LimelightIOReal(LimelightIOConfig config, DoubleSupplier yawSupplier) {
        this.config = config;
        this.yawSupplier = yawSupplier;
        if (config.useInternalIMU && !config.isLimelight4) {
            throw new IllegalArgumentException("Internal IMU only exists on limelight 4");
        } else if (config.useInternalIMU) {
            LimelightHelpers.SetIMUAssistAlpha(config.name, config.filterAlpha);
        }
    }

    /**
     * The reliability score ranges from 0 to 1 (inclusive) and indicates the trustworthiness of
     * this limelight's prediction accounting its prediction weight.
     *
     * @return 0-1 (inclusive). 1 is the most reliable and 0 the least.
     */
    @Override
    public double getReliabilityScore(LimelightHelpers.PoseEstimate poseEstimate) {
        if (poseEstimate.tagCount == 0) {
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

    @Override
    public void updateInputs(LimelightIOInputs inputs) {
        if (config.isLimelight4) {
            // can use internal IMU
            if (RobotState.isDisabled()) {
                // use IMU mode 1 - seed internal IMU with data
                LimelightHelpers.SetIMUMode(config.name, InternalIMUMode.EXTERNAL_SEED.getValue());
            } else {
                // enabled - use IMU mode 4 - externally assisted internal IMU MegaTag2
                LimelightHelpers.SetIMUMode(
                        config.name, InternalIMUMode.INTERNAL_EXTERNAL_ASSIST.getValue());
            }
        }
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
        inputs.pose = new Pose3d(estimate.pose);
        inputs.timestampSeconds = estimate.timestampSeconds;
        inputs.latency = estimate.latency;
        inputs.detectedTagIds = Arrays.stream(estimate.rawFiducials).mapToInt(r -> r.id).toArray();
        inputs.reliability = getReliabilityScore(estimate);
    }
}
