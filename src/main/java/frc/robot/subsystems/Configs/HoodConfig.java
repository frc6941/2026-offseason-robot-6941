package frc.robot.subsystems.Configs;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.RobotConstants.CANIVORE_CAN_BUS;

import com.ctre.phoenix6.signals.InvertedValue;
import lib.ironpulse.subsystem.SubsystemConfig;
import lib.ntext.NTParameter;

public class HoodConfig {
    private HoodConfig() {}

    public static final String NAME = "Hood";
    public static final int HOOD_MOTOR_MAIN_ID = 52;
    public static final double HOOD_GEAR_RATIO = 48.0 / 8.0;
    // Local hardware constants
    public static final SubsystemConfig HOOD_CONFIG =
            SubsystemConfig.builder()
                    .name(NAME)
                    .mainBus(CANIVORE_CAN_BUS)
                    .mainId(HOOD_MOTOR_MAIN_ID)
                    .motorInvertedValue(InvertedValue.Clockwise_Positive)
                    .SensorToMechanismRatio(HOOD_GEAR_RATIO)
                    .zeroingConfig(
                            SubsystemConfig.ZeroingConfig.builder()
                                    .zeroingCurrentLimit(50)
                                    .zeroingFilterSize(5)
                                    .zeroingVoltage(1)
                                    .build())
                    // TODO: set this so the target angle represents the Real angle of the hood
                    .zeroOffset(Degrees.of(0.0))
                    .simConfig(
                            SubsystemConfig.SimConfig.builder().gearRatio(HOOD_GEAR_RATIO).build())
                    .build();

    @NTParameter(tableName = "Params/" + NAME)
    public static final class HoodParams {

        public static final double kP = 3.75;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kV = 0.1308;
        public static final double kA = 0.0068;
        public static final double kS = 0.13;

        // Motion Magic
        public static final double motionMagicVelRPS = 250.0;
        public static final double motionMagicAccelRPS2 = 600.0;
        public static final double motionMagicJerkRPS3 = 0.0;

        // Tolerances / behavior
        public static final double atGoalToleranceDegrees = 1;
    }
}
