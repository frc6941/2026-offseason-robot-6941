package frc.robot.subsystems.Configs;

import static frc.robot.RobotConstants.ROBORIO_CAN_BUS;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import lib.ironpulse.subsystem.SubsystemConfig;
import lib.ntext.NTParameter;

public class IntakeConfig {

    public static final String INTAKER_ROLLER_NAME = "IntakerRoller";
    public static final String INTAKER_DOWN_ROLLER_NAME = "IntakerDownRoller";
    public static final String INTAKER_EXTENSION_NAME = "IntakerExtension";


    //id
    private static final int INTAKER_ROLLER_MOTOR_MAIN_ID = 0;
    private static final int INTAKER_ROLLER_MOTOR_FOLLOWER_ID = 1;
    private static final int INTAKER_DOWN_ROLLER_MOTOR_MAIN_ID = 2;
    private static final int INTAKER_EXTENSION_MOTOR_MAIN_ID = 3;


    private static final double INTAKER_ROLLER_GEAR_RATIO = 12.0 / 42.0;
    private static final double INTAKER_ROLLER_MOTOR_RAMP = 1;

    private static final double INTAKER_DOWN_ROLLER_GEAR_RATIO = 1.0 / 1.5;
    private static final double INTAKER_DOWN_ROLLER_MOTOR_RAMP = 1;

    private static final double INTAKER_EXTENSION_GEAR_RATIO = 3.0 / 10.0;
    public static final double INTAKE_EXTENSION_METERS_PER_ROTATION = 0.10511;

    public static final SubsystemConfig INTAKER_ROLLER_CONFIG =
            SubsystemConfig.builder()
                    .name(INTAKER_ROLLER_NAME)
                    .mainBus(ROBORIO_CAN_BUS)
                    .mainId(INTAKER_ROLLER_MOTOR_MAIN_ID)
                    .motorInvertedValue(InvertedValue.Clockwise_Positive)
                    .defaultBrake(true)
                    .SensorToMechanismRatio(INTAKER_ROLLER_GEAR_RATIO)
                    .ramp(INTAKER_ROLLER_MOTOR_RAMP)
                    .followers(
                            new SubsystemConfig.FollowerConfig[] {
                                SubsystemConfig.FollowerConfig.builder()
                                        .id(INTAKER_ROLLER_MOTOR_FOLLOWER_ID)
                                        .bus(ROBORIO_CAN_BUS)
                                        .opposeMain(MotorAlignmentValue.Opposed)
                                        .ramp(INTAKER_ROLLER_MOTOR_RAMP)
                                        .build()
                            })
                    .build();

    public static final SubsystemConfig INTAKER_DOWN_ROLLER_CONFIG =
            SubsystemConfig.builder()
                    .name(INTAKER_DOWN_ROLLER_NAME)
                    .mainBus(ROBORIO_CAN_BUS)
                    .mainId(INTAKER_DOWN_ROLLER_MOTOR_MAIN_ID)
                    .motorInvertedValue(InvertedValue.CounterClockwise_Positive)
                    .defaultBrake(true)
                    .SensorToMechanismRatio(INTAKER_DOWN_ROLLER_GEAR_RATIO)
                    .ramp(INTAKER_DOWN_ROLLER_MOTOR_RAMP)
                    .build();

    public static final SubsystemConfig INTAKER_EXTENSION_CONFIG =
            SubsystemConfig.builder()
                    .name(INTAKER_EXTENSION_NAME)
                    .mainBus(ROBORIO_CAN_BUS)
                    .mainId(INTAKER_EXTENSION_MOTOR_MAIN_ID)
                    .motorInvertedValue(InvertedValue.CounterClockwise_Positive)
                    .defaultBrake(true)
                    .SensorToMechanismRatio(INTAKER_EXTENSION_GEAR_RATIO)
                    .zeroingConfig(
                            SubsystemConfig.ZeroingConfig.builder()
                                    .zeroingCurrentLimit(30)
                                    .zeroingFilterSize(5)
                                    .zeroingVoltage(-2)
                                    .build())
                    .build();

    @NTParameter(tableName = "Params/" + INTAKER_ROLLER_NAME)
    public static final class IntakerRollerParams {
        public static final double intakeVelRPS = -20;
        public static final double outtakeVelRPS = 10;

        public static final double slowIntakeMultiplier = 0.6;
        public static final double fastIntakeMultiplier = 1.0;

        public static final double kP = 0.1;
        public static final double kI = 0.0;
        public static final double kD = 0;

        public static final double kV = 0.12;
        public static final double kA = 0.0;
        public static final double kS = 0.1;

        public static final double intakeRPSStep = 2.0;
        public static final double intakeRPSStepSeconds = 0.1;
        public static final double intakeStopRPSStep = 2.0;
        public static final double intakeStopRPSStepSeconds = 0.25;
    }

    @NTParameter(tableName = "Params/" + INTAKER_EXTENSION_NAME)
    public static final class IntakerExtensionParams {
        public static final double kP = 15;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kV = 0.1308;
        public static final double kA = 0.0068;
        public static final double kS = 0.13;

        public static final double motionMagicVelRPS = 1000.0;
        public static final double motionMagicAccelRPS2 = 150.0;
        public static final double motionMagicJerkRPS3 = 0.0;

        public static final double atGoalToleranceMeters = 0.01;
        public static final double deployPosMeters = 0.316;
        public static final double retractedFeedPosMeters = 0.05;
        public static final double feedPosMeters = 0.17;
        public static final double retractPosMeters = 0.01;

        public static final double feedOscillationRateHz = 2;
        public static final boolean isBrake = false;
    }

    private IntakeConfig() {}
}
