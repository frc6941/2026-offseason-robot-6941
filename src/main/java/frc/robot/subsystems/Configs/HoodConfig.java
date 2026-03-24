package frc.robot.subsystems.Configs;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.RobotConstants.ROBORIO_CAN_BUS;

import com.ctre.phoenix6.signals.InvertedValue;
import lib.ironpulse.subsystem.SubsystemConfig;
import lib.ntext.NTParameter;

public class HoodConfig {
    public static final String NAME = "Hood";
    public static final int HOOD_MOTOR_MAIN_ID = 52;
    public static final double HOOD_GEAR_RATIO = 194.0 / 12.0 * 48.0 / 8.0;
    public static final double HOOD_ANGLE_ZEROED_DEG = 12.8;
    public static final double HOOD_DOF_DEG = 36;
    // Local hardware constants
    public static final SubsystemConfig HOOD_CONFIG =
            SubsystemConfig.builder()
                    .name(NAME)
                    .mainBus(ROBORIO_CAN_BUS)
                    .mainId(HOOD_MOTOR_MAIN_ID)
                    .motorInvertedValue(InvertedValue.Clockwise_Positive)
                    .SensorToMechanismRatio(HOOD_GEAR_RATIO)
                    .statorCurrentLimitAmps(30)
                    .supplyCurrentLimitAmps(25)
                    .zeroingConfig(
                            SubsystemConfig.ZeroingConfig.builder()
                                    .zeroingCurrentLimit(20)
                                    .zeroingFilterSize(5)
                                    .zeroingVoltage(-1)
                                    .build())
                    .forwardSoftLimitDegrees(Degrees.of(HOOD_ANGLE_ZEROED_DEG + HOOD_DOF_DEG))
                    .reverseSoftLimitDegrees(Degrees.of(0))
                    .zeroOffset(Degrees.of(HOOD_ANGLE_ZEROED_DEG))
                    .simConfig(
                            SubsystemConfig.SimConfig.builder().gearRatio(HOOD_GEAR_RATIO).build())
                    .build();

    private HoodConfig() {}

    @NTParameter(tableName = "Params/" + NAME)
    public static final class HoodParams {

        public static final double kP = 300;
        public static final double kI = 5;
        public static final double kD = 1;
        public static final double kV = 0.35;
        public static final double kA = 0;
        public static final double kS = 0.25;

        // Motion Magic
        public static final double motionMagicVelRPS = 250.0;
        public static final double motionMagicAccelRPS2 = 600.0;
        public static final double motionMagicJerkRPS3 = 0.0;

        public static final double testAngle = 20;

        // Tolerances / behavior
        public static final double atGoalToleranceDegrees = 1;
    }
}
