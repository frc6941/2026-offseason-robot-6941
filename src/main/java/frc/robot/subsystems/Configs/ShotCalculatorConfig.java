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
        public static final double trajectoryBiasDegGOAL = -2;
        public static final double trajectoryBiasDegFEED = -2;
        public static final double loookfwdDistanceScale = 1;
        public static final double lateralVelocityCompScale = 1.2;
        public static final double lookfwdDelayCycles = 0;
        public static final double lookfwdFlightScale = 0;
        // Two-lane switching for close-range deconfliction.
        public static final double laneSwitchEnabled = 0;
        public static final double laneHalfSeparationMeters = 0.1;
        public static final double laneSwitchCycles = 15.0;
        public static final double laneSwitchNearDistanceMeters = 2.5;

        public static final double lookfwdMinCycles = 0.0;
        public static final double lookfwdMaxCycles = 100.0;

        // Linear mapping (actuator-space)
        // rpm = rpmA * exitSpeed + rpmB * bbaDeg + rpmC
        public static final double rpmA = 250;
        public static final double rpmB = 0;
        public static final double rpmC = 170;

        // hood = hoodB * launchAngle + hoodC
        public static final double hoodB = 1;
        public static final double hoodC = 0.0;
    }
}
