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
  private final DeviationParamSources deviationParams;
  private boolean isPrevDisabled = true;

  public LimelightIOReal(
      LimelightIOConfig config,
      DoubleSupplier yawSupplier,
      BooleanSupplier rejectionSupplier,
      DeviationParamSources deviationParams) {
    this.config = config;
    this.yawSupplier = yawSupplier;
    this.rejectionSupplier = rejectionSupplier;
    this.deviationParams = deviationParams;
    if (useInternalIMU()) {
      LimelightHelpers.SetIMUAssistAlpha(config.getName(), config.getFilterAlpha());
      setIMUMode();
    }
  }

  private boolean isLimelight4() {
    return config.getLimeLight4Config() != null;
  }

  private boolean useInternalIMU() {
    return isLimelight4() && config.getLimeLight4Config().isUseInternalIMU();
  }

  @Override
  public boolean canUseInternalIMU() {
    return config.getMountPosition() == LimelightIOConfig.MountPosition.ON_ROBOT
        && useInternalIMU();
  }

  /**
   * The reliability score ranges from 0 to 1 (inclusive) and indicates the trustworthiness of this
   * limelight's prediction accounting its prediction weight.
   *
   * @return 0-1 (inclusive). 1 is the most reliable and 0 the least.
   */
  @Override
  public double getReliabilityScore(LimelightHelpers.PoseEstimate poseEstimate) {
    if (poseEstimate.tagCount == 0 || rejectionSupplier.getAsBoolean()) {
      // no tags or decided to reject
      return 0;
    } else if (poseEstimate.tagCount == 1) {
      return 0.5 * config.getWeight();
    } else {
      // tags >= 2
      return 1 * config.getWeight();
    }
  }

  @Override
  public String getName() {
    return config.getName();
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
      LimelightHelpers.SetIMUMode(config.getName(), InternalIMUMode.EXTERNAL_ONLY.getValue());
      return;
    }
    if (RobotState.isDisabled()) {
      // disabled; use IMU mode 1 - seed internal IMU with data
      LimelightHelpers.SetIMUMode(config.getName(), InternalIMUMode.EXTERNAL_SEED.getValue());
    } else {
      // enabled - use IMU mode 4 - externally assisted internal IMU MegaTag2
      LimelightHelpers.SetIMUMode(
          config.getName(), InternalIMUMode.INTERNAL_EXTERNAL_ASSIST.getValue());
    }
  }

  @Override
  public void setLEDMode(LEDMode mode) {
    switch (mode) {
      case ON -> LimelightHelpers.setLEDMode_ForceOn(config.getName());
      case OFF -> LimelightHelpers.setLEDMode_ForceOff(config.getName());
      case BLINK -> LimelightHelpers.setLEDMode_ForceBlink(config.getName());
      case PIPELINE_CONTROL -> LimelightHelpers.setLEDMode_PipelineControl(config.getName());
    }
  }

  @Override
  public int getPipeline() {
    return (int) LimelightHelpers.getCurrentPipelineIndex(config.getName());
  }

  @Override
  public void setPipeline(int pipeline) {
    LimelightHelpers.setPipelineIndex(config.getName(), pipeline);
  }

  @Override
  public void setThrottle(boolean robotEnabled) {
    if (!isLimelight4()) {
      return;
    }
    LimelightHelpers.SetThrottle(
        config.getName(),
        robotEnabled
            ? config.getLimeLight4Config().getThrottleWhenEnabled()
            : config.getLimeLight4Config().getThrottleWhenDisabled());
  }

  @Override
  public void setAprilTagIdFilter(int[] ids) {
    LimelightHelpers.SetFiducialIDFiltersOverride(config.getName(), ids);
  }

  @Override
  public void updateInputs(LimelightIOInputs inputs) {
    if (canUseInternalIMU() && isPrevDisabled != RobotState.isDisabled()) {
      setIMUMode();
    }
    isPrevDisabled = RobotState.isDisabled();
    LimelightHelpers.SetRobotOrientation(
        config.getName(),
        yawSupplier.getAsDouble(),
        0,
        0,
        0,
        0,
        0); // the last 5 parameters are not necessary

    // generate pose Estimate
    LimelightHelpers.PoseEstimate estimate;
    if (config.isUseMegaTag2()) {
      estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(config.getName());
    } else {
      estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(config.getName());
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
  public double[] getVisionStdDevComponents(double reliability) {
    return new double[] {
      deviationParams.xStdDev() * (2 - reliability),
      deviationParams.yStdDev() * (2 - reliability),
      deviationParams.zStdDev() * (2 - reliability),
      deviationParams.angleStdDev() * (2 - reliability)
    };
  }

  @Override
  public double getImuCorrectionReliabilityThreshold() {
    return deviationParams.imuCorrectionReliabilityThreshold();
  }

  @Override
  public double getIMUYawInternal() {
    return LimelightHelpers.getIMUData(config.getName()).Yaw;
  }

  @Override
  public double getIMUYawRobot() {
    return LimelightHelpers.getIMUData(config.getName()).robotYaw;
  }
}
