package lib.ironpulse.limelight;

import lombok.Builder;

/** Builder-style configuration for limelight IO. */
@Builder
public class LimelightIOConfig {
    public enum MountPosition {
        ON_ROBOT,
        ON_MECHANISM
    }

    @Builder.Default public final boolean useMegaTag2 = true;
    // weight given to its estimation
    // ranges from 0-1 inclusive.
    @Builder.Default public final double weight = 1.0;
    // complementary filter alpha for the internal IMU. Default is 0.001.
    // Lower values (e.g., 0.001): Smoother, slower drift correction. The internal IMU is trusted
    // more.
    // Higher values (e.g., 0.01): Faster tracking of the reference source (MT1 or external IMU).
    @Builder.Default public final double filterAlpha = 0.001;
    @Builder.Default public final double xStdDev = 0.7;
    @Builder.Default public final double yStdDev = 0.7;
    @Builder.Default public final double zStdDev = 1.0;
    @Builder.Default public final double angleStdDev = 9999999;
    @Builder.Default public final double imuCorrectionReliabilityThreshold = 0.9;

    public final String name;
    public final boolean isLimelight4;
    public final MountPosition mountPosition;

    @Builder.ObtainVia(field = "isLimelight4")
    public final boolean useInternalIMU;
}
