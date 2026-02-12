package frc.robot.subsystems.Configs;

import static frc.robot.RobotConstants.CANIVORE_CAN_BUS;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import lib.ironpulse.subsystem.SubsystemConfig;
import lib.ntext.NTParameter;

public class IntakerRollerConfig {
    public static final String NAME = "IntakerRoller";
    private static final int INTAKER_ROLLER_MOTOR_MAIN_ID = 32;
    private static final double INTAKER_ROLLER_GEAR_RATIO = 12.0 / 26.0;
    public static final SubsystemConfig INTAKER_ROLLER_CONFIG =
            SubsystemConfig.builder()
                    .name(NAME)
                    .mainBus(CANIVORE_CAN_BUS)
                    .mainId(INTAKER_ROLLER_MOTOR_MAIN_ID)
                    .motorInvertedValue(InvertedValue.CounterClockwise_Positive)
                    .defaultBrake(false)
                    .kSValue(StaticFeedforwardSignValue.UseVelocitySign)
                    .SensorToMechanismRatio(INTAKER_ROLLER_GEAR_RATIO)
                    .simConfig(
                            SubsystemConfig.SimConfig.builder()
                                    .gearRatio(INTAKER_ROLLER_GEAR_RATIO)
                                    .build())
                    .build();

    private IntakerRollerConfig() {}

    @NTParameter(tableName = "Params/" + NAME)
    public static final class IntakerRollerParams {
        // velocity gains
        public static final double kP = 1;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kV = 0.1308;
        public static final double kA = 0.0068;
        public static final double kS = 0.13;

        public static final double velocityAtGoalToleranceRPS = 30;

        public static final double testVelRPS = 110;
        public static final double intakeVelRPS = 50;
        public static final double outtakeVelRPS = -50;
        public static final double idleVelRPS = 0;
    }
}
