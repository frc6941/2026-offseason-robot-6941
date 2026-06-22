package frc.robot.subsystems.Configs;

import static frc.robot.RobotConstants.ROBORIO_CAN_BUS;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import lib.ironpulse.subsystem.SubsystemConfig;
import lib.ntext.NTParameter;

public class IntakeConfig {

    public static final String INTAKER_ROLLER_NAME = "IntakerRoller";

    private static final int INTAKER_ROLLER_MOTOR_MAIN_ID = 0;
    private static final int INTAKER_ROLLER_MOTOR_FOLLOWER_ID = 1;

    private static final double INTAKER_ROLLER_GEAR_RATIO = 12.0 / 42.0;

    public static final SubsystemConfig INTAKER_ROLLER_CONFIG =
            SubsystemConfig.builder()
                    .name(INTAKER_ROLLER_NAME)
                    .mainBus(ROBORIO_CAN_BUS)
                    .mainId(INTAKER_ROLLER_MOTOR_MAIN_ID)
                    .motorInvertedValue(InvertedValue.Clockwise_Positive)
                    .defaultBrake(true)
                    .SensorToMechanismRatio(INTAKER_ROLLER_GEAR_RATIO)
                    .ramp(0.5)
                    .followers(
                            new SubsystemConfig.FollowerConfig[] {
                                SubsystemConfig.FollowerConfig.builder()
                                        .id(INTAKER_ROLLER_MOTOR_FOLLOWER_ID)
                                        .bus(ROBORIO_CAN_BUS)
                                        .opposeMain(MotorAlignmentValue.Opposed)
                                        .ramp(0.5)
                                        .build()
                            })
                    .build();

    public static final String INTAKER_DOWN_ROLLER_NAME = "IntakerDownRoller";

    private static final int INTAKER_DOWN_ROLLER_MOTOR_MAIN_ID = 2;

    private static final double INTAKER_DOWN_ROLLER_GEAR_RATIO = 1 / 1.5;

    public static final SubsystemConfig INTAKER_DOWN_ROLLER_CONFIG =
            SubsystemConfig.builder()
                    .name(INTAKER_DOWN_ROLLER_NAME)
                    .mainBus(ROBORIO_CAN_BUS)
                    .mainId(INTAKER_DOWN_ROLLER_MOTOR_MAIN_ID)
                    .motorInvertedValue(InvertedValue.CounterClockwise_Positive)
                    .defaultBrake(true)
                    .SensorToMechanismRatio(INTAKER_DOWN_ROLLER_GEAR_RATIO)
                    .ramp(0.5)
                    .build();

    @NTParameter(tableName = "Params/" + INTAKER_ROLLER_NAME)
    public static final class IntakerRollerParams {
        public static final double intakeVelRPS = 10;
        public static final double outtakeVelRPS = -10;

        public static final double slowIntakeMultiplier = 0.6;
        public static final double fastIntakeMultiplier = 1.0;

        public static final double kP = 0.1;
        public static final double kI = 0.0;
        public static final double kD = 0;

        public static final double kV = 0.12;
        public static final double kA = 0.0;
        public static final double kS = 0.1;

        public static final double intakeStartRPS = -2.0;
        public static final double intakeMaxRPS = -10.0;
        public static final double intakeRPSStep = 2.0;
        public static final double intakeRPSStepSeconds = 0.5;
        public static final double intakeStopRPSStep = 2.0;
        public static final double intakeStopRPSStepSeconds = 0.25;
    }

    private IntakeConfig() {}
}
