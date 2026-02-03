package frc.robot.subsystems.Configs;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import frc.robot.RobotConstants;
import lib.ironpulse.subsystem.SubsystemConfig;
import lib.ntext.NTParameter;

public class ShooterConfig {
    private ShooterConfig() {}

    public static final String NAME = "Shooter";
    public static final String CANIVORE_CAN_BUS_NAME = RobotConstants.CANIVORE_CAN_BUS_NAME;

    private static final int SHOOTER_MOTOR_MAIN_ID = 50;
    private static final int SHOOTER_MOTOR_FOLLOWER_ID = 51;
    private static final int SHOOTER_GEAR_RATIO = 1;

    public static final SubsystemConfig SHOOTER_CONFIG =
            SubsystemConfig.builder()
                    .name(NAME)
                    .mainBus(CANIVORE_CAN_BUS_NAME)
                    .mainId(SHOOTER_MOTOR_MAIN_ID)
                    .motorInvertedValue(InvertedValue.CounterClockwise_Positive)
                    .defaultBrake(false)
                    .kSValue(StaticFeedforwardSignValue.UseVelocitySign)
                    .followers(
                            new SubsystemConfig.FollowerConfig[] {
                                SubsystemConfig.FollowerConfig.builder()
                                        .id(SHOOTER_MOTOR_FOLLOWER_ID)
                                        .bus(CANIVORE_CAN_BUS_NAME)
                                        .opposeMain(MotorAlignmentValue.Opposed)
                                        .build()
                            })
                    .SensorToMechanismRatio(SHOOTER_GEAR_RATIO)
                    .simConfig(
                            SubsystemConfig.SimConfig.builder()
                                    .gearRatio(SHOOTER_GEAR_RATIO)
                                    .build())
                    .build();

    @NTParameter(tableName = "Params/" + NAME)
    public static final class ShooterParams {
        // velocity gains
        public static final double kP = 0.1;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kV = 0.11625;
        public static final double kA = 0.0031037;
        public static final double kS = 0.12346;

        public static final double velocityAtGoalToleranceRPS = 30;

        public static final double testVelRPS = 110;
        public static final double idleVelRPS = 50;
    }
}
