package lib.ironpulse.subsystem;

import static edu.wpi.first.units.Units.Degree;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Builder.Default;

/**
 * Builder-style configuration container for a TalonFX-based MotorIO implementation. Mirrors the
 * convenience of {@link lib.ironpulse.swerve.SwerveConfig} for mechanism motors.
 *
 * <p>Intent: - Store all device IDs/buses and Phoenix configurations in one place - Optionally
 * enable a remote CANcoder (raw/sync/fused) for feedback processing inside the IO - Optionally
 * configure one or more follower TalonFX devices
 */
@Builder
public class SubsystemConfig {
    // Naming (for logs/diagnostics)
    @Default public final String name = "UNNAMED";

    // Primary TalonFX
    public final int mainId;
    public final String mainBus;

    /** Phoenix configuration to apply to the primary TalonFX. */
    @Default public final TalonFXConfiguration fxConfig = new TalonFXConfiguration();

    // Optional remote CANcoder feedback (handled internally by IO when enabled)
    @Default public final boolean enableRemoteCANcoder = false;
    public final RemoteCANcoder remoteCANcoder; // nullable unless enableRemoteCANcoder is true

    // Optional followers
    @Default public final FollowerConfig[] followers = new FollowerConfig[0];

    @Default
    public final InvertedValue motorInvertedValue = InvertedValue.CounterClockwise_Positive;

    @Default public final int filterSize = 1;
    @Default public final double SensorToMechanismRatio = 1.0;

    @Default public final boolean updateOutputs = true;

    // other close loop configs
    @Default public final GravityTypeValue gravityType = GravityTypeValue.Elevator_Static;

    @Default
    public final StaticFeedforwardSignValue kSValue = StaticFeedforwardSignValue.UseClosedLoopSign;

    /** Soft-limit enables at init; thresholds should be part of fxConfig if used. */
    // TODO: soft limit
    @Default public final boolean enableForwardSoftLimit = false;

    @Default public final boolean enableReverseSoftLimit = false;

    /** ONLY used when needing to offset the zero position to use the CTRE Kg for arm cos */
    @Default public final Angle zeroOffset = Degree.of(0.0);

    @Default public final ZeroingConfig zeroingConfig = ZeroingConfig.builder().build();

    /** Optional remote CANcoder configuration summary. */
    @Builder
    @AllArgsConstructor
    public static class RemoteCANcoder {
        public final int id;
        public final String bus;
        public final double magnetOffset;
        public final double rotorToSensorRatio;

        @Default
        public final SensorDirectionValue sensorDirection =
                SensorDirectionValue.CounterClockwise_Positive;

        @Default
        public final FeedbackSensorSourceValue feedbackSensorSource =
                FeedbackSensorSourceValue.FusedCANcoder;

        @Default public final boolean useContinousWrap = false;
    }

    @Builder
    @AllArgsConstructor
    public static class ZeroingConfig {
        @Default public double zeroingCurrentLimit = 40.0;
        @Default public double zeroingVoltage = -2.0;
        @Default public int zeroingFilterSize = 5;
    }

    /** Optional follower TalonFX configuration. */
    @Builder
    @AllArgsConstructor
    public static class FollowerConfig {
        public final int id;
        public final String bus;

        /** If true, follower output is inverted (oppose main). */
        @Default public final boolean opposeMain = false;
    }

    @Builder
    @AllArgsConstructor
    public static class SimConfig {
        @Default public final double gearRatio = 1.0d;

        @Default
        public final TrapezoidProfile.Constraints profile =
                new TrapezoidProfile.Constraints(50.0, 100.0);
    }

    @Default public SimConfig simConfig = SimConfig.builder().build();
}
