package lib.ironpulse.limelight;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose3d;
import lib.ironpulse.utils.LimelightHelpers;
import lombok.Getter;

public interface LimelightIO {
    // reliability score: from 0 to 1
    double getReliabilityScore(LimelightHelpers.PoseEstimate poseEstimate);

    default void setPipeline(int pipeline) {}

    default int getPipeline() {
        return 0;
    }

    default void setLEDMode(LEDMode mode) {}

    default void setAprilTagIdFilter(int[] ids) {}

    default void clearAprilTagIdFilter() {
        setAprilTagIdFilter(new int[0]);
    }

    default void updateInputs(LimelightIOInputs inputs) {}

    default double[] getVisionStdDevComponents(double reliability) {
        return new double[] {
            0.7 * (2 - reliability),
            0.7 * (2 - reliability),
            1.0 * (2 - reliability),
            9999999 * (2 - reliability)
        };
    }

    default double getImuCorrectionReliabilityThreshold() {
        return 0.9;
    }

    // FIXME: leave only one function for yaw. This was to see that whether robotYaw equals to
    // internal yaw.
    double getIMUYawInternal();

    double getIMUYawRobot();

    boolean canUseInternalIMU();

    default String getName() {
        return "UNNAMED";
    }

    @AutoLog
    class LimelightIOInputs {
        public Pose3d pose;
        public double timestampSeconds;
        public double latency;
        public double reliability;
        public int[] detectedTagIds;
    }

    enum InternalIMUMode {
        // no internal IMU - only swerve IMU
        EXTERNAL_ONLY(0),
        // internal IMU is seeded (calibrated) by the external IMU.
        // N.B. still uses external IMU for pose estimation.
        EXTERNAL_SEED(1),
        // only internal IMU used
        INTERNAL_ONLY(2),
        // internal + MegaTag1 assisted IMU
        INTERNAL_MT1_ASSIST(3),
        // internal + external IMU for MegaTag2
        INTERNAL_EXTERNAL_ASSIST(4);

        @Getter private final int value;

        InternalIMUMode(int value) {
            this.value = value;
        }
    }

    enum LEDMode {
        PIPELINE_CONTROL,
        ON,
        OFF,
        BLINK
    }
}
