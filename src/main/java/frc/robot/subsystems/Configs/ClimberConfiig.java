package frc.robot.subsystems.Configs;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.RobotConstants.CANIVORE_CAN_BUS;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.units.measure.Distance;
import lib.ironpulse.subsystem.SubsystemConfig;
import lib.ntext.NTParameter;

public class ClimberConfiig {
    public static final String CLIMBER_NAME = "Climber";
    public static final Distance CLIMB_METERS_PER_ROTATION = Meters.of(0.116);
    private static final int CLIMBER_MOTOR_MAIN_ID = 60;
    private static final double CLIMBER_GEAR_RATIO = 25 * 50 / 34;
    private static final double CLIMBER_STATOR_CURRENT_LIMIT_AMPS = 40;
    private static final double CLIMBER_SUPPLY_CURRENT_LIMIT_AMPS = 40;

    public static final SubsystemConfig CLIMBER_CONFIG =
            SubsystemConfig.builder()
                    .name(CLIMBER_NAME)
                    .mainBus(CANIVORE_CAN_BUS)
                    .mainId(CLIMBER_MOTOR_MAIN_ID)
                    .motorInvertedValue(InvertedValue.CounterClockwise_Positive)
                    .defaultBrake(true)
                    .kSValue(StaticFeedforwardSignValue.UseClosedLoopSign)
                    .SensorToMechanismRatio(CLIMBER_GEAR_RATIO)
                    .statorCurrentLimitAmps(CLIMBER_STATOR_CURRENT_LIMIT_AMPS)
                    .supplyCurrentLimitAmps(CLIMBER_SUPPLY_CURRENT_LIMIT_AMPS)
                    .simConfig(
                            SubsystemConfig.SimConfig.builder()
                                    .gearRatio(CLIMBER_GEAR_RATIO)
                                    .build())
                    .zeroingConfig(
                            SubsystemConfig.ZeroingConfig.builder()
                                    .zeroingCurrentLimit(35)
                                    .zeroingFilterSize(5)
                                    .zeroingVoltage(-2)
                                    .build())
                    .build();

    private ClimberConfiig() {}

    @NTParameter(tableName = "Params/" + CLIMBER_NAME)
    public static final class ClimberParams {

        public static final double kP = 15;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kV = 0.1308;
        public static final double kA = 0.0068;
        public static final double kS = 0.13;

        // Motion Magic
        public static final double motionMagicVelRPS = 1000.0;
        public static final double motionMagicAccelRPS2 = 150.0;
        public static final double motionMagicJerkRPS3 = 0.0;

        // Tolerances / behavior
        public static final double atGoalToleranceMeters = 0.01;
        public static final double bottomMeters = 0.01;
        public static final double climbedMeters = 0.2;
        public static final double simpleClimbMeters = 0.1;

        public static final boolean isBrake = false;
    }
}
