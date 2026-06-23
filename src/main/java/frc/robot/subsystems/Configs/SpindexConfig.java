package frc.robot.subsystems.Configs;

import static frc.robot.RobotConstants.CANIVORE_CAN_BUS;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import lib.ironpulse.subsystem.SubsystemConfig;
import lib.ntext.NTParameter;

public class SpindexConfig {

    private static final CANBus CANBUS = CANIVORE_CAN_BUS;

    private static final int SPINDEXER_ID = 4;
    private static final int SPINDEXER_FOLLOWER_ID = 5;

    public static final String SPINDEXER_NAME = "spindexer";
    private static final double SPINDEXER_GEAR_RATIO = 1.0 / 1.0;

    public static final SubsystemConfig SPINDEXER_CONFIG =
            SubsystemConfig.builder()
                    .name(SPINDEXER_NAME)
                    .mainId(SPINDEXER_ID)
                    .mainBus(CANBUS)
                    .SensorToMechanismRatio(SPINDEXER_GEAR_RATIO)
                    .defaultBrake(true)
                    .motorInvertedValue(InvertedValue.Clockwise_Positive)
                    .simConfig(
                            SubsystemConfig.SimConfig.builder()
                                    .gearRatio(SPINDEXER_GEAR_RATIO)
                                    .build())
                    .followers(
                            new SubsystemConfig.FollowerConfig[] {
                                SubsystemConfig.FollowerConfig.builder()
                                        .id(SPINDEXER_ID)
                                        .bus(CANBUS)
                                        .opposeMain(MotorAlignmentValue.Aligned)
                                        .build()
                            })
                    .build();

    @NTParameter(tableName = "Params/" + SPINDEXER_NAME)
    public static final class SpindexerParams {

        public static final double kP = 0.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;

        public static final double kV = 0.0;
        public static final double kA = 0.0;
        public static final double kS = 0.0;
    }
}
