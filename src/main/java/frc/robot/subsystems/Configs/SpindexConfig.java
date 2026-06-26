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
    private static final int SPINDEXER_FOLLOWER1_ID = 5;
    private static final int SPINDEXER_FOLLOWER2_ID = 6;

    public static final String SPINDEXER_NAME = "spindexer";
    private static final double SPINDEXER_GEAR_RATIO = 1.0 / 1.0;

    public static final SubsystemConfig SPINDEXER_CONFIG =
            SubsystemConfig.builder()
                    .name(SPINDEXER_NAME)
                    .mainId(SPINDEXER_ID)
                    .mainBus(CANBUS)
                    .SensorToMechanismRatio(SPINDEXER_GEAR_RATIO)
                    .defaultBrake(true)
                    .motorInvertedValue(InvertedValue.CounterClockwise_Positive)
                    .simConfig(
                            SubsystemConfig.SimConfig.builder()
                                    .gearRatio(SPINDEXER_GEAR_RATIO)
                                    .build())
                    .followers(
                            new SubsystemConfig.FollowerConfig[] {
                                SubsystemConfig.FollowerConfig.builder()
                                        .id(SPINDEXER_FOLLOWER1_ID)
                                        .bus(CANBUS)
                                        .opposeMain(MotorAlignmentValue.Opposed)
                                        .build()
                            })
                    .followers(
                            new SubsystemConfig.FollowerConfig[] {
                                SubsystemConfig.FollowerConfig.builder()
                                        .id(SPINDEXER_FOLLOWER2_ID)
                                        .bus(CANBUS)
                                        .opposeMain(MotorAlignmentValue.Opposed)
                                        .build()
                            })
                    .build();

    @NTParameter(tableName = "Params/IdxModes")
    public static final class SpindexerModeParams {
        public static final double feedRPS = 2.7; // 1.6 for 10541

        public static final double revRPS = -1;
        public static final double idleRPS = 0.0;
    }

    @NTParameter(tableName = "Params/" + SPINDEXER_NAME)
    public static final class SpindexerParams {

        public static final double kP = 0.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;

        public static final double kV = 0.0;
        public static final double kA = 0.0;
        public static final double kS = 0.0;

        public static final double unjammTimeoutSec = 0.16;
        public static final double unjammTriggerAmps = 60;
        public static final double unjammTriggerBelowRps = 1.4; // 1.5 for 10541
        public static final double unjammTriggerSec = 0.08;
        public static final double unjammLockoutSec = 0.2;
        public static final boolean periodicUnjamEnabled = false;
        public static final double periodicUnjamIntervalSec = 1.0;
        public static final boolean unjamBeforeFeedEnabled = false;

        public static final boolean isBrake = true;

        public static final double velocityAtGoalToleranceRPS = 0.1;
    }
}
