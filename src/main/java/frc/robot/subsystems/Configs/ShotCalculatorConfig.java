package frc.robot.subsystems.Configs;

import static frc.robot.RobotConstants.is10541;

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
        public static final double trajectoryBiasDegGOAL = is10541 ? -6 : -6;
        public static final double trajectoryBiasDegFEED = -2;
        public static final double loookfwdDistanceScale = 0;
        public static final double lateralVelocityCompScale = is10541 ? 1.01 : 1.2;
        public static final double lookfwdDelayCycles = is10541 ? 1 : 3;

        public static final double lookfwdFlightScale = 0;

        public static final double lookfwdMinCycles = 0.0;
        public static final double lookfwdMaxCycles = 20.0;

        // Linear mapping (actuator-space)
        // rpm = rpmA * exitSpeed + rpmB * bbaDeg + rpmC
        public static final double rpmA = is10541 ? 337 : 237.1;
        public static final double rpmB = is10541 ? 0 : 0;
        public static final double rpmC = is10541 ? -368.8 : 146;

        public static final double distanceScaler = 1.00;
    }
}
