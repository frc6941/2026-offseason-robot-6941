package frc.robot.subsystems.Configs;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.units.measure.Angle;
import frc.robot.RobotConstants;
import lib.ironpulse.subsystem.SubsystemConfig;
import lib.ntext.NTParameter;

public class TurretConfig {
    private TurretConfig() {}

    public static final String NAME = "Turret";
    public static final String CANIVORE_CAN_BUS_NAME = RobotConstants.CANIVORE_CAN_BUS_NAME;

    public static final int TURRET_MOTOR_MAIN_ID = 96;
    public static final double TURRET_GEAR_RATIO = 40.0;

    public static final int TURRET_ENCODER_G1_ID = 95;
    public static final int TURRET_ENCODER_G2_ID = 94;

    public static final Angle TURRET_ENCODER_G1_OFFSET = Degrees.of(0.0);
    public static final Angle TURRET_ENCODER_G2_OFFSET = Degrees.of(0.0);

    // Zeroing Coder constants
    public static final int G0_TOOTH_COUNT = 70;
    public static final int G1_TOOTH_COUNT = 36;
    public static final int G2_TOOTH_COUNT = 34;

    public static final Angle ENCODER_DELTA_WRAP_THRESHOLD = Degrees.of(250.0);
    public static final Angle ANGLE_CORRECTION_THRESHOLD = Degrees.of(100.0);

    public static final SubsystemConfig TURRET_CONFIG =
            SubsystemConfig.builder()
                    .name(NAME)
                    .mainBus(CANIVORE_CAN_BUS_NAME)
                    .mainId(TURRET_MOTOR_MAIN_ID)
                    .SensorToMechanismRatio(TURRET_GEAR_RATIO)
                    .motorInvertedValue(InvertedValue.Clockwise_Positive)
                    .kSValue(StaticFeedforwardSignValue.UseVelocitySign)
                    .forwardSoftLimitDegrees(Degrees.of(360))
                    .reverseSoftLimitDegrees(Degrees.of(-360.0))
                    .statorCurrentLimitAmps(80)
                    .supplyCurrentLimitAmps(80)
                    .simConfig(
                            SubsystemConfig.SimConfig.builder()
                                    .gearRatio(TURRET_GEAR_RATIO)
                                    .build())
                    .build();

    @NTParameter(tableName = "Params/" + NAME)
    public static final class TurretParams {
        // velocity gains
        // IMPORTANT: Makesure we tune these first before tuning the position gains
        // Velocity trackeing should be clean and accurate
        public static final double kP = 3.75;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kV = 0.1308;
        public static final double kA = 0.0068;
        public static final double kS = 0.13;

        // shold be small
        public static final double kpPos = 0.01;
        public static final double kiPos = 0.0;
        public static final double kdPos = 0.0;

        public static final double maxVelocityRPS = 5;
        public static final double maxAccelerationRPS2 = 10;

        public static final double kchassisVelCompensation = 1;

        // Tolerances / behavior
        public static final double velocityAtGoalToleranceRPS = 1;
        public static final double positionAtGoalToleranceDegrees = 1;
    }
}
