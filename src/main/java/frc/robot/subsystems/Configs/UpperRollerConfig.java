package frc.robot.subsystems.Configs;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import frc.robot.RobotConstants;
import lib.ironpulse.subsystem.SubsystemConfig;
import lib.ntext.NTParameter;

public class UpperRollerConfig {
    private UpperRollerConfig() {}

    public static final String NAME = "UpperRoller";
    public static final String CANIVORE_CAN_BUS_NAME = RobotConstants.CANIVORE_CAN_BUS_NAME;

    private static final int UPPER_ROLLER_MOTOR_MAIN_ID = 9;
    private static final double UPPER_ROLLER_GEAR_RATIO = 3;

    public static final SubsystemConfig UPPER_ROLLER_CONFIG =
            SubsystemConfig.builder()
                    .name(NAME)
                    .mainBus(CANIVORE_CAN_BUS_NAME)
                    .mainId(UPPER_ROLLER_MOTOR_MAIN_ID)
                    .motorInvertedValue(InvertedValue.Clockwise_Positive)
                    .defaultBrake(false)
                    .kSValue(StaticFeedforwardSignValue.UseVelocitySign)
                    .SensorToMechanismRatio(UPPER_ROLLER_GEAR_RATIO)
                    .simConfig(
                            SubsystemConfig.SimConfig.builder()
                                    .gearRatio(UPPER_ROLLER_GEAR_RATIO)
                                    .build())
                    .build();

    @NTParameter(tableName = "Params/" + NAME)
    public static final class UpperRollerParams {
        // velocity gains
        public static final double kP = 1;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kV = 0.11473;
        public static final double kA = 0.0075714;
        public static final double kS = 0.0019217;

        public static final double velocityAtGoalToleranceRPS = 30;

        public static final double testVelRPS = 110;
        public static final double idleVelRPS = 50;
    }
}
