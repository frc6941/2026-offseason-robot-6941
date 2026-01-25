package lib.ironpulse.limelight;

import edu.wpi.first.math.geometry.Pose3d;
import lib.ironpulse.utils.LimelightHelpers;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLog;

public interface LimelightIO {
    // reliability score: from 0 to 1
    double getReliabilityScore(LimelightHelpers.PoseEstimate poseEstimate);

    default void updateInputs(LimelightIOInputs inputs) {}

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
        // internal + external IMU for MegaTag1 - not recommended
        INTERNAL_MT1_ASSIST_DO_NOT_USE(3),
        // internal + external IMU for MegaTag2
        INTERNAL_EXTERNAL_ASSIST(4);

        @Getter private final int value;

        InternalIMUMode(int value) {
            this.value = value;
        }
    }
}
