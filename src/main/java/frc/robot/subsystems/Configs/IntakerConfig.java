package frc.robot.subsystems.Configs;

import static frc.robot.RobotConstants.CANIVORE_BUS;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import lib.ironpulse.subsystem.SubsystemConfig;
import lib.ntext.NTParameter;

public class IntakerConfig {
    private IntakerConfig() {}

    public static final String NAME = "Intaker";

    private static final int INTAKER_MOTOR_MAIN_ID = 50;
    private static final double INTAKER_GEAR_RATIO = 3;

    public static final SubsystemConfig INTAKER_CONFIG =
            SubsystemConfig.builder()
                    .name(NAME)
                    .mainBus(CANIVORE_BUS)
                    .mainId(INTAKER_MOTOR_MAIN_ID)
                    .motorInvertedValue(InvertedValue.Clockwise_Positive)
                    .defaultBrake(false)
                    .kSValue(StaticFeedforwardSignValue.UseVelocitySign)
                    .SensorToMechanismRatio(INTAKER_GEAR_RATIO)
                    .simConfig(
                            SubsystemConfig.SimConfig.builder()
                                    .gearRatio(INTAKER_GEAR_RATIO)
                                    .build())
                    .build();

    @NTParameter(tableName = "Params/" + NAME)
    public static final class IntakerParams {
        // velocity gains
        public static final double kP = 1;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kV = 0.1308;
        public static final double kA = 0.0068;
        public static final double kS = 0.13;

        public static final double velocityAtGoalToleranceRPS = 30;

        public static final double testVelRPS = 110;
        public static final double idleVelRPS = 50;
    }
}
