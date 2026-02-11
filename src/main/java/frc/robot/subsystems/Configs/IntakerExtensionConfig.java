package frc.robot.subsystems.Configs;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.RobotConstants.CANIVORE_CAN_BUS;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import lib.ironpulse.subsystem.SubsystemConfig;
import lib.ntext.NTParameter;

public class IntakerExtensionConfig {
    public static final String NAME = "IntakerExtension";
    private static final int INTAKER_EXTENSION_MOTOR_MAIN_ID = 33;
    private static final double INTAKER_EXTENSION_GEAR_RATIO = 40.0 / 26.0;
    public static final SubsystemConfig INTAKER_EXTENSION_CONFIG =
            SubsystemConfig.builder()
                    .name(NAME)
                    .mainBus(CANIVORE_CAN_BUS)
                    .mainId(INTAKER_EXTENSION_MOTOR_MAIN_ID)
                    .motorInvertedValue(InvertedValue.Clockwise_Positive)
                    .defaultBrake(false)
                    .kSValue(StaticFeedforwardSignValue.UseVelocitySign)
                    .SensorToMechanismRatio(INTAKER_EXTENSION_GEAR_RATIO)
                    .simConfig(
                            SubsystemConfig.SimConfig.builder()
                                    .gearRatio(INTAKER_EXTENSION_GEAR_RATIO)
                                    .build())
                    .zeroingConfig(
                            SubsystemConfig.ZeroingConfig.builder()
                                    .zeroingCurrentLimit(50)
                                    .zeroingFilterSize(5)
                                    .zeroingVoltage(1)
                                    .build())
                    .zeroOffset(Degrees.of(0.0))
                    .build();

    private IntakerExtensionConfig() {}

    @NTParameter(tableName = "Params/" + NAME)
    public static final class IntakerExtensionParams {

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
        public static final double atGoalToleranceMeters = 0.01;
        public static final double deployPosMeters = 0.2;
        public static final double retractPosMeters = 0.0;
    }
}
