package frc.robot.subsystems.Configs;

import lib.ironpulse.limelight.DeviationParamSources;
import lib.ironpulse.limelight.LimelightIOConfig;
import lib.ntext.NTParameter;

public class LimeLightConfig {
    public static final String NAME = "LimeLight";
    public static final LimelightIOConfig limelight1Config =
            LimelightIOConfig.builder()
                    .name(NAME)
                    .mountPosition(LimelightIOConfig.MountPosition.ON_ROBOT)
                    .limeLight4Config(LimelightIOConfig.Limelight4Config.builder().build())
                    .build();

    public static DeviationParamSources asDeviationParams() {
        return new DeviationParamSources() {
            @Override
            public double xStdDev() {
                return LimelightParamsNT.xStdDev.getValue();
            }

            @Override
            public double yStdDev() {
                return LimelightParamsNT.yStdDev.getValue();
            }

            @Override
            public double zStdDev() {
                return LimelightParamsNT.zStdDev.getValue();
            }

            @Override
            public double angleStdDev() {
                return LimelightParamsNT.angleStdDev.getValue();
            }

            @Override
            public double imuCorrectionReliabilityThreshold() {
                return LimelightParamsNT.imuCorrectionReliabilityThreshold.getValue();
            }
        };
    }

    @NTParameter(tableName = "Params/" + NAME)
    public static final class LimelightParams {
        public static final double xStdDev = 0.7;
        public static final double yStdDev = 0.7;
        public static final double zStdDev = 1.0;
        public static final double angleStdDev = 9999999.0;
        public static final double imuCorrectionReliabilityThreshold = 0.9;
    }
}
