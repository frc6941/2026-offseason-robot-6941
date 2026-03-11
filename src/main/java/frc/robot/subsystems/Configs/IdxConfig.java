package frc.robot.subsystems.Configs;

import static frc.robot.RobotConstants.CANIVORE_CAN_BUS;
import static frc.robot.RobotConstants.is10541;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import lib.ironpulse.subsystem.SubsystemConfig;
import lib.ntext.NTParameter;

public class IdxConfig {
    public static final String SPINDEXER = "Spindexer";
    public static final double STATOR_CURRENT_LIMIT_AMPS = 90;
    public static final double SUPPLY_CURRENT_LIMIT_AMPS = 90;
    public static final double SPINDEXER_GEAR_RATIO =
            is10541
                    ? 16 / 1 * 30 / 22 * 42 / 20
                    : 9.0 / 1.0 * 34 / 18 * 42.0 / 20.0; // RATIO FROM MOTOR TO SPIN
    private static final int SPINDEXER_ID = 58;

    private static final int SPINDEXER_FOLLOWER_ID = 59;
    public static final SubsystemConfig SPINDEXER_CFG =
            SubsystemConfig.builder()
                    .name(SPINDEXER)
                    .mainBus(CANIVORE_CAN_BUS)
                    .mainId(SPINDEXER_ID)
                    .motorInvertedValue(InvertedValue.Clockwise_Positive)
                    .defaultBrake(true)
                    .kSValue(StaticFeedforwardSignValue.UseVelocitySign)
                    .SensorToMechanismRatio(SPINDEXER_GEAR_RATIO)
                    .simConfig(
                            SubsystemConfig.SimConfig.builder()
                                    .gearRatio(SPINDEXER_GEAR_RATIO)
                                    .build())
                    .followers(
                            new SubsystemConfig.FollowerConfig[] {
                                SubsystemConfig.FollowerConfig.builder()
                                        .id(SPINDEXER_FOLLOWER_ID)
                                        .bus(CANIVORE_CAN_BUS)
                                        .opposeMain(MotorAlignmentValue.Aligned)
                                        .statorCurrentLimitAmps(STATOR_CURRENT_LIMIT_AMPS)
                                        .supplyCurrentLimitAmps(SUPPLY_CURRENT_LIMIT_AMPS)
                                        .build()
                            })
                    .statorCurrentLimitAmps(STATOR_CURRENT_LIMIT_AMPS)
                    .supplyCurrentLimitAmps(SUPPLY_CURRENT_LIMIT_AMPS)
                    .build();

    private IdxConfig() {}

    @NTParameter(tableName = "Params/IdxModes")
    public static final class SpindexerModeParams {
        public static final double feedRPS = 2; // 1.6 for 10541

        public static final double revRPS = -1;
        public static final double idleRPS = 0.0;
    }

    @NTParameter(tableName = "Params/" + SPINDEXER)
    public static final class SpindexerParams {
        // TODO: sysId
        public static final double kP = 10;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kV = 3.9;
        public static final double kA = 1;
        public static final double kS = 0.25;

        public static final double unjammTimeoutSec = 0.15;
        public static final double unjammTriggerAmps = 65;
        public static final double unjammTriggerBelowRps = 1.8; // 1.5 for 10541
        public static final double unjammTriggerSec = 0.08;
        public static final double unjammLockoutSec = 0.2;

        public static final boolean isBrake = true;

        public static final double velocityAtGoalToleranceRPS = 0.01;
    }
}
