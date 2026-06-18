package frc.robot.subsystems.Configs;

import static frc.robot.RobotConstants.CANIVORE_CAN_BUS;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import lib.ironpulse.subsystem.SubsystemConfig;
import lib.ntext.NTParameter;

public class IntakeConfig {

    public static final String INTAKER_ROLLER_NAME = "IntakerRoller";

    private static final int INTAKER_ROLLER_MOTOR_MAIN_ID = 32;
    private static final int INTAKER_ROLLER_MOTOR_FOLLOWER_ID = 35;

    private static final double INTAKER_ROLLER_GEAR_RATIO = 26.0 / 12.0;

    public static final SubsystemConfig INTAKER_ROLLER_CONFIG =
            SubsystemConfig.builder()
                    .name(INTAKER_ROLLER_NAME)
                    .mainBus(CANIVORE_CAN_BUS)
                    .mainId(INTAKER_ROLLER_MOTOR_MAIN_ID)
                    .motorInvertedValue(InvertedValue.Clockwise_Positive)
                    .defaultBrake(true)
                    .SensorToMechanismRatio(INTAKER_ROLLER_GEAR_RATIO)
                    .followers(
                            new SubsystemConfig.FollowerConfig[] {
                                SubsystemConfig.FollowerConfig.builder()
                                        .id(INTAKER_ROLLER_MOTOR_FOLLOWER_ID)
                                        .bus(CANIVORE_CAN_BUS)
                                        .opposeMain(MotorAlignmentValue.Opposed)
                                        .build()
                            })
                    .build();

    public static final String INTAKER_DOWN_ROLLER_NAME = "IntakerDownRoller";

    private static final int INTAKER_DOWN_ROLLER_MOTOR_MAIN_ID = 33;

    private static final double INTAKER_DOWN_ROLLER_GEAR_RATIO = 1.0;

    public static final SubsystemConfig INTAKER_DOWN_ROLLER_CONFIG =
            SubsystemConfig.builder()
                    .name(INTAKER_DOWN_ROLLER_NAME)
                    .mainBus(CANIVORE_CAN_BUS)
                    .mainId(INTAKER_DOWN_ROLLER_MOTOR_MAIN_ID)
                    .motorInvertedValue(InvertedValue.CounterClockwise_Positive)
                    .defaultBrake(true)
                    .SensorToMechanismRatio(INTAKER_DOWN_ROLLER_GEAR_RATIO)
                    .build();

    @NTParameter(tableName = "Params/" + INTAKER_ROLLER_NAME)
    public static final class IntakerRollerParams {

        public static final double kP = 40;
        public static final double kI = 0.005;
        public static final double kD = 0;

        public static final double kV = 0.3;
        public static final double kA = 0.0068;
        public static final double kS = 0.13;

        public static final double intakeVelRPS = 52;
        public static final double outtakeVelRPS = -50;
        public static final double idleVelRPS = 0;
    }

    private IntakeConfig() {}
}
