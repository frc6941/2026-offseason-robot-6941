package frc.robot.subsystems.Configs;

import static frc.robot.RobotConstants.ROBORIO_CAN_BUS;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import lib.ironpulse.subsystem.SubsystemConfig;
import lib.ntext.NTParameter;

public class ShooterConfig {

    public static final String SHOOTER_NAME = "Shooter";

    private static final int SHOOTER_MOTOR_MAIN_ID = 6;
    private static final int SHOOTER_MOTOR_FOLLOWER_ID = 7;

    private static final double SHOOTER_GEAR_RATIO = 15.0 / 17.0;

    public static final SubsystemConfig SHOOTER_CONFIG =
            SubsystemConfig.builder()
                    .name(SHOOTER_NAME)
                    .mainBus(ROBORIO_CAN_BUS)
                    .mainId(SHOOTER_MOTOR_MAIN_ID)
                    .motorInvertedValue(InvertedValue.CounterClockwise_Positive)
                    .followers(
                            new SubsystemConfig.FollowerConfig[] {
                                SubsystemConfig.FollowerConfig.builder()
                                        .id(SHOOTER_MOTOR_FOLLOWER_ID)
                                        .bus(ROBORIO_CAN_BUS)
                                        .opposeMain(MotorAlignmentValue.Opposed)
                                        .build()
                            })
                    .SensorToMechanismRatio(SHOOTER_GEAR_RATIO)
                    .simConfig(
                            SubsystemConfig.SimConfig.builder()
                                    .gearRatio(SHOOTER_GEAR_RATIO)
                                    .build())
                    .build();

    @NTParameter(tableName = "Params/" + SHOOTER_NAME)
    public class ShooterParams {

        public static final double kP = 0.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;

        public static final double kV = 0.0;
        public static final double kA = 0.0;
        public static final double kS = 0.0;

        public static final double runVelRPS = 20;
        public static final double idleVelRPS = 10;
    }
}
