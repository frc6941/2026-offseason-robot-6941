package frc.robot.subsystems.Configs;

import lib.ntext.NTParameter;

/**
 * NetworkTables-backed tuning parameters for the shot calculator.
 *
 * <p>These values are read directly by {@code ShotCalculator} on each call so no local refresh
 * cache is required.
 */
public final class ShotCalculatorConfig {
    private ShotCalculatorConfig() {}

    @NTParameter(tableName = "Params/ShotCalculator")
    public static final class ShotCalculatorParams {
        // Model tuning (model-space adjustments)
        public static final double speedScale = 1.0;
        public static final double speedOffsetMps = 0.0;
        public static final double angleOffsetDeg = 0.0;
        public static final double trajectoryBiasDeg = 0.0;

        // Linear mapping (actuator-space)
        // rpm = rpmA * exitSpeed + rpmC
        public static final double rpmA = 1;
        public static final double rpmC = 0.0;

        // hood = hoodB * launchAngle + hoodC
        public static final double hoodB = 1;
        public static final double hoodC = 0.0;
    }
}
