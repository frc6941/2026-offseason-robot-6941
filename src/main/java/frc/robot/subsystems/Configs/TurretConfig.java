package frc.robot.subsystems.Configs;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.RobotConstants.CANIVORE_CAN_BUS;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.units.measure.Angle;
import lib.ironpulse.subsystem.SubsystemConfig;
import lib.ntext.NTParameter;

public class TurretConfig {
    private TurretConfig() {}

    public static final String NAME = "Turret";

    public static final int TURRET_MOTOR_MAIN_ID = 20;
    public static final double TURRET_GEAR_RATIO = 34 / 8 * 88 / 11;

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

    public static final Angle TURRET_SOFT_LIMIT = Degrees.of(220.0);
    public static final Angle TURRET_SOFT_LIMIT_MARGIN = Degrees.of(5.0);

    public static final SubsystemConfig TURRET_CONFIG =
            SubsystemConfig.builder()
                    .name(NAME)
                    .mainBus(CANIVORE_CAN_BUS)
                    .mainId(TURRET_MOTOR_MAIN_ID)
                    .SensorToMechanismRatio(TURRET_GEAR_RATIO)
                    .motorInvertedValue(InvertedValue.CounterClockwise_Positive)
                    .kSValue(StaticFeedforwardSignValue.UseVelocitySign)
                    .defaultBrake(true)
                    .forwardSoftLimitDegrees(TURRET_SOFT_LIMIT)
                    .reverseSoftLimitDegrees(TURRET_SOFT_LIMIT.unaryMinus())
                    .statorCurrentLimitAmps(80)
                    .supplyCurrentLimitAmps(80)
                    .simConfig(
                            SubsystemConfig.SimConfig.builder()
                                    .gearRatio(TURRET_GEAR_RATIO)
                                    .build())
                    .build();

    @NTParameter(tableName = "Params/" + NAME + "Vel")
    public static final class TurretVelParams {
        // velocity gains
        // IMPORTANT: Makesure we tune these first before tuning the position gains
        // Velocity trackeing should be clean and accurate
        public static final double kP = 2.5;
        public static final double kI = 0.0;
        public static final double kD = 0.1;
        public static final double kV = 3.0112;
        public static final double kA = 0.2948;
        public static final double kS = 0.23;
        // Tolerances / behavior
        public static final double velocityAtGoalToleranceRPS = 1;
        public static final boolean isBrake = true;
    }

    @NTParameter(tableName = "Params/" + NAME + "Pos")
    public static final class TurretPosParams {
        // shold be small
        public static final double kpSeek = 1;
        public static final double kiSeek = 0.0;
        public static final double kdSeek = 0.0;

        public static final double kpTrack = 1;
        public static final double kiTrack = 0.0;
        public static final double kdTrack = 0.0;

        public static final double maxVelocityRPS = 5;
        public static final double maxAccelerationRPS2 = 10;
        public static final double kchassisVelCompensation = 1;
        public static final double positionAtGoalToleranceDegrees = 1;
        // Hysteresis thresholds for automatic mode switching.
        // Must satisfy seekEnterErrorDegrees > trackEnterErrorDegrees.
        public static final double seekEnterErrorDegrees = 15;
        public static final double trackEnterErrorDegrees = 5;
    }
}
