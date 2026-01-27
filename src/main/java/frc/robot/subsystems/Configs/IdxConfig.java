package frc.robot.subsystems.Configs;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import frc.robot.RobotConstants;
import lib.ironpulse.subsystem.SubsystemConfig;
import lib.ntext.NTParameter;

public class IdxConfig {
    private IdxConfig() {}

    public static final String SPIN = "IdxSpin";
    public static final String VERT = "IdxVert";
    public static final String HORIZ = "IdxHoriz";
    public static final String CANIVORE_CAN_BUS_NAME = RobotConstants.CANIVORE_CAN_BUS_NAME;

    private static final int SPIN_ID = 2;
    private static final int VERT_ID = 33;
    private static final int HORIZ_ID = 3;
    private static final double SPIN_GEAR_RATIO = 176.0 / 11.0 * 46.0 / 22.0;
    private static final double VERT_GEAR_RATIO = 1;
    private static final double HORIZ_GEAR_RATIO = 1;

    public static final SubsystemConfig SPIN_CFG =
            SubsystemConfig.builder()
                    .name(SPIN)
                    .mainBus(CANIVORE_CAN_BUS_NAME)
                    .mainId(SPIN_ID)
                    .motorInvertedValue(InvertedValue.Clockwise_Positive)
                    .defaultBrake(true)
                    .kSValue(StaticFeedforwardSignValue.UseVelocitySign)
                    .SensorToMechanismRatio(SPIN_GEAR_RATIO)
                    .simConfig(
                            SubsystemConfig.SimConfig.builder().gearRatio(SPIN_GEAR_RATIO).build())
                    .build();

    public static final SubsystemConfig VERT_CFG =
            SubsystemConfig.builder()
                    .name(VERT)
                    .mainBus(CANIVORE_CAN_BUS_NAME)
                    .mainId(VERT_ID)
                    .motorInvertedValue(InvertedValue.Clockwise_Positive)
                    .defaultBrake(true)
                    .kSValue(StaticFeedforwardSignValue.UseVelocitySign)
                    .SensorToMechanismRatio(VERT_GEAR_RATIO)
                    .simConfig(
                            SubsystemConfig.SimConfig.builder().gearRatio(VERT_GEAR_RATIO).build())
                    .build();

    public static final SubsystemConfig HORIZ_CFG =
            SubsystemConfig.builder()
                    .name(HORIZ)
                    .mainBus(CANIVORE_CAN_BUS_NAME)
                    .mainId(HORIZ_ID)
                    .motorInvertedValue(InvertedValue.Clockwise_Positive)
                    .defaultBrake(false)
                    .kSValue(StaticFeedforwardSignValue.UseVelocitySign)
                    .SensorToMechanismRatio(HORIZ_GEAR_RATIO)
                    .simConfig(
                            SubsystemConfig.SimConfig.builder().gearRatio(HORIZ_GEAR_RATIO).build())
                    .build();

    @NTParameter(tableName = "Params/IdxModes")
    public static final class IdxModeParams {
        public static final double feedSpin = 110.0;
        public static final double feedVert = 110.0;
        public static final double feedHoriz = 110.0;

        public static final double revSpin = -110.0;
        public static final double revVert = -110.0;
        public static final double revHoriz = -110.0;
    }

    @NTParameter(tableName = "Params/IdxSpin")
    public static final class IdxSpinParams {
        public static final double kP = 1;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kV = 0.1308;
        public static final double kA = 0.0068;
        public static final double kS = 0.13;

        public static final double velocityAtGoalToleranceRPS = 30;

    }

    @NTParameter(tableName = "Params/IdxVert")
    public static final class IdxVertParams {
        public static final double kP = 1;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kV = 0.1308;
        public static final double kA = 0.0068;
        public static final double kS = 0.13;

        public static final double velocityAtGoalToleranceRPS = 30;

    }

    @NTParameter(tableName = "Params/IdxHoriz")
    public static final class IdxHorizParams {
        public static final double kP = 1;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kV = 0.1308;
        public static final double kA = 0.0068;
        public static final double kS = 0.13;

        public static final double velocityAtGoalToleranceRPS = 30;
    }
}
