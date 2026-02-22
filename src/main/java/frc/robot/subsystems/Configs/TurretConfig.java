package frc.robot.subsystems.Configs;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.RobotConstants.ROBORIO_CAN_BUS;
import static frc.robot.RobotConstants.is10541;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.units.measure.Angle;
import lib.ironpulse.subsystem.SubsystemConfig;
import lib.ntext.NTParameter;

public class TurretConfig {
    public static final String NAME = "Turret";

    public static final int TURRET_MOTOR_MAIN_ID = 31;
    public static final double TURRET_GEAR_RATIO = 34.0 / 8.0 * 88.0 / 11.0;

    public static final int TURRET_ENCODER_G1_ID = 59;
    public static final int TURRET_ENCODER_G2_ID = 58;
    public static final Angle TURRET_ENCODER_G1_OFFSET =
            Rotations.of(is10541 ? 0.344970703125 : -0.14013671875);
    public static final Angle TURRET_ENCODER_G2_OFFSET =
            Rotations.of(is10541 ? -0.003662109375 : -0.659423828125);
    // Zeroing Coder constants
    public static final int G0_TOOTH_COUNT = 88;
    public static final int G1_TOOTH_COUNT = 16;
    public static final int G2_TOOTH_COUNT = 15;
    public static final Angle ENCODER_DELTA_WRAP_THRESHOLD = Degrees.of(190);
    public static final Angle ANGLE_CORRECTION_THRESHOLD = Degrees.of(30);
    // Zero offset- the position at which the two encoders are set to zero.
    public static final Angle TURRET_ZERO_OFFSET = Degrees.of(-138.4);
    public static final Angle TURRET_DOF = Degrees.of(225);
    public static final Angle TURRET_SOFT_LIMIT_CCW = TURRET_DOF.plus(TURRET_ZERO_OFFSET);
    public static final Angle TURRET_SOFT_LIMIT_CW =
            TURRET_DOF.unaryMinus().plus(TURRET_ZERO_OFFSET);

    public static final Angle TURRET_SOFT_LIMIT_MARGIN = Degrees.of(5.0);
    public static final SubsystemConfig TURRET_CONFIG =
            SubsystemConfig.builder()
                    .name(NAME)
                    .mainBus(ROBORIO_CAN_BUS)
                    .mainId(TURRET_MOTOR_MAIN_ID)
                    .SensorToMechanismRatio(TURRET_GEAR_RATIO)
                    .motorInvertedValue(InvertedValue.CounterClockwise_Positive)
                    .kSValue(StaticFeedforwardSignValue.UseVelocitySign)
                    .defaultBrake(true)
                    .forwardSoftLimitDegrees(TURRET_SOFT_LIMIT_CCW)
                    .reverseSoftLimitDegrees(TURRET_SOFT_LIMIT_CW)
                    .statorCurrentLimitAmps(80)
                    .supplyCurrentLimitAmps(80)
                    .simConfig(
                            SubsystemConfig.SimConfig.builder()
                                    .gearRatio(TURRET_GEAR_RATIO)
                                    .build())
                    .build();

    private TurretConfig() {}

    @NTParameter(tableName = "Params/" + NAME + "Vel")
    public static final class TurretVelParams {
        // velocity gains
        // IMPORTANT: Makesure we tune these first before tuning the position gains
        // Velocity trackeing should be clean and accurate
        public static final double kP = 150;
        public static final double kI = 0.0;
        public static final double kD = 0;
        public static final double kV = 0;
        public static final double kA = 0;
        public static final double kS = 0;
        // Magnitude of turret gravity-like bias compensation in torque current (A).
        // Sign is computed in TurretSubsystem based on whether commanded motion moves
        // away from or toward TURRET_ZERO_OFFSET.
        public static final double turretBiasCompTCAmps = 10;
        public static final double turretBiasCompSetpointStepFlipDeg = 0.5;
        public static final double turretBiasCompZeroOffsetDeadBandDeg = 1.0;
        // Tolerances / behavior
        public static final double velocityAtGoalToleranceRPS = 1;
        public static final boolean isBrake = true;
    }

    @NTParameter(tableName = "Params/" + NAME + "Pos")
    public static final class TurretPosParams {
        // shold be small
        public static final double kpSeek = 4.5;
        public static final double kiSeek = 0.0;
        public static final double kdSeek = 0.0;

        public static final double kpTrack = 8.5;
        public static final double kiTrack = 0.0;
        public static final double kdTrack = 0.0;

        public static final double maxVelocityRPS = 3.5;
        public static final double maxAccelerationRPS2 = 1;
        public static final double kchassisVelCompensation = 1;
        public static final double positionAtGoalToleranceDegrees = 1;
        // Hysteresis thresholds for automatic mode switching.
        // Must satisfy seekEnterErrorDegrees > trackEnterErrorDegrees.
        public static final double seekEnterErrorDegrees = 15;
        public static final double trackEnterErrorDegrees = 5;
    }
}
