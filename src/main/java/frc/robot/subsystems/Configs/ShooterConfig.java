package frc.robot.subsystems.Configs;

import static frc.robot.RobotConstants.CANIVORE_CAN_BUS;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import lib.ironpulse.subsystem.SubsystemConfig;
import lib.ntext.NTParameter;

public class ShooterConfig {
    public static final String NAME = "Shooter";
    private static final int SHOOTER_MOTOR_MAIN_ID = 50;
    private static final int SHOOTER_MOTOR_FOLLOWER_ID = 51;
    private static final double SHOOTER_GEAR_RATIO = 24.0 / 24.0;
    public static final SubsystemConfig SHOOTER_CONFIG =
            SubsystemConfig.builder()
                    .name(NAME)
                    .mainBus(CANIVORE_CAN_BUS)
                    .mainId(SHOOTER_MOTOR_MAIN_ID)
                    .motorInvertedValue(InvertedValue.CounterClockwise_Positive)
                    .defaultBrake(false)
                    .kSValue(StaticFeedforwardSignValue.UseVelocitySign)
                    .followers(
                            new SubsystemConfig.FollowerConfig[] {
                                SubsystemConfig.FollowerConfig.builder()
                                        .id(SHOOTER_MOTOR_FOLLOWER_ID)
                                        .bus(CANIVORE_CAN_BUS)
                                        .opposeMain(MotorAlignmentValue.Aligned)
                                        .build()
                            })
                    .SensorToMechanismRatio(SHOOTER_GEAR_RATIO)
                    .simConfig(
                            SubsystemConfig.SimConfig.builder()
                                    .gearRatio(SHOOTER_GEAR_RATIO)
                                    .build())
                    .build();

    private ShooterConfig() {}

    @NTParameter(tableName = "Params/" + NAME)
    public static final class ShooterParams {
        // velocity gains

        public static final double kP = 0.4;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kV = 0.2;
        public static final double kA = 0.0;
        public static final double kS = 0.0;

        public static final double velocityAtGoalToleranceRPS = 1;

        public static final double testVelRPS = 10;
        public static final double idleVelRPS = 0;
    }
}
