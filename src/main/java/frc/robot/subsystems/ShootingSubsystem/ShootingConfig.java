package frc.robot.subsystems.ShootingSubsystem;

import lib.ntext.NTParameter;

public final class ShootingConfig {
    public ShootingConfig() {}

    // Standalone presets (not part of distance interpolation)
    @NTParameter(tableName = "Params/ShooterTable")
    @SuppressWarnings("unused")
    private static final class ShootingPresetParams {
        static final double customFWV = 500.0;
        static final double customBBA = 20.0;

        // Interpolated table points: (distance meters) -> (flywheel velocity rpm, backboard angle
        // deg)
        //
        // NOTE: This is a fixed-size table for the @NTParameter framework. Add/remove points by
        // editing these fields (P1..PN).
        static final double P1_DIS = 2.0;
        static final double P1_FWV = 1850.0;
        static final double P1_BBA = 10.0;

        static final double P2_DIS = 2.5;
        static final double P2_FWV = 1950.0;
        static final double P2_BBA = 15.0;

        static final double P3_DIS = 3.0;
        static final double P3_FWV = 1950.0;
        static final double P3_BBA = 21.0;

        static final double P4_DIS = 4.0;
        static final double P4_FWV = 2200.0;
        static final double P4_BBA = 23.7;

        static final double P5_DIS = 4.5;
        static final double P5_FWV = 2300.0;
        static final double P5_BBA = 28.5;

        static final double P6_DIS = 5.0;
        static final double P6_FWV = 2400.0;
        static final double P6_BBA = 28.5;

        static final double P7_DIS = 5.5;
        static final double P7_FWV = 2525.0;
        static final double P7_BBA = 28.5;

        static final double P8_DIS = 6.0;
        static final double P8_FWV = 2650.0;
        static final double P8_BBA = 29.5;

        static final double P9_DIS = 6.5;
        static final double P9_FWV = 2800.0;
        static final double P9_BBA = 29.7;

        static final double P10_DIS = 7.0;
        static final double P10_FWV = 2900.0;
        static final double P10_BBA = 30.0;
    }
}
