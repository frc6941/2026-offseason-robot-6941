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
        public static final double trajectoryBiasDegGOAL = -3.5; // -2.3 for 10541
        public static final double trajectoryBiasDegFEED = -2;
        public static final double loookfwdDistanceScale = 1.12;
        public static final double lateralVelocityCompScale = 1; // 1 for 10541
        public static final double lookfwdDelayCycles = 1;
        public static final double lookfwdFlightScale = 0;

        public static final double lookfwdMinCycles = 0.0;
        public static final double lookfwdMaxCycles = 20.0;

        // Linear mapping (actuator-space)
        // rpm = rpmA * exitSpeed + rpmB * bbaDeg + rpmC
        public static final double rpmA = 409; // 245 for 10541
        public static final double rpmB = 0; // 2.3 for 10541
        public static final double rpmC = -843;

        public static final double distanceScaler = 1.05;
    }
}
