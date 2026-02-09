package lib.ironpulse.limelight;

import lombok.Builder;

@Builder
public class LimelightSubsystemConfig {
    @Builder.Default public final double xStdDev = 0.7;
    @Builder.Default public final double yStdDev = 0.7;
    @Builder.Default public final double zStdDev = 1.0;
    @Builder.Default public final double angleStdDev = 9999999;
    @Builder.Default public final double imuCorrectionReliabilityThreshold = 0.9;
}
