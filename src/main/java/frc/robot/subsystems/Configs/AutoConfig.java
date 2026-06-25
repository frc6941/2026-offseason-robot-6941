package frc.robot.subsystems.Configs;

import lib.ntext.NTParameter;

public final class AutoConfig {
    private AutoConfig() {}

    @NTParameter(tableName = "Params/Auto")
    public static final class AutoParams {
        public static final double translationKP = 5.0;
        public static final double translationKI = 0.0;
        public static final double translationKD = 0.0;
        public static final double rotationKP = 3.0;
        public static final double rotationKI = 0.0;
        public static final double rotationKD = 0.0;
        public static final double shootTimeoutSec = 1.5;
    }
}
