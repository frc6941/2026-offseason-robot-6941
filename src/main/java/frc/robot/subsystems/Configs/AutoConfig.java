package frc.robot.subsystems.Configs;

import lib.ntext.NTParameter;

public class AutoConfig {
    @NTParameter(tableName = "Params/auto")
    public static final class AutoParams {
        public static final class AutoPoseParams {
            public static final double kpStrave = 4;
            public static final double kiStrave = 0.0;
            public static final double kdStrave = 0.0;
            public static final double kpSpin = 4;
            public static final double kiSpin = 0.0;
            public static final double kdSpin = 0.0;
        }

        public static final class AutoPathParams {
            public static final double kpStrave = 5;
            public static final double kiStrave = 0.0;
            public static final double kdStrave = 0.0;
            public static final double kpSpin = 3;
            public static final double kiSpin = 0.0;
            public static final double kdSpin = 0.1;
        }
    }
}
