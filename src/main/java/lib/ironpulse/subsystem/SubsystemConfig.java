package lib.ironpulse.subsystem;

import static edu.wpi.first.units.Units.KilogramSquareMeters;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.MomentOfInertia;
import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Builder.Default;

/**
 * Builder-style configuration container for a TalonFX-based MotorIO
 * implementation.
 * Mirrors the convenience of {@link lib.ironpulse.swerve.SwerveConfig} for
 * mechanism motors.
 *
 * Intent:
 * - Store all device IDs/buses and Phoenix configurations in one place
 * - Optionally enable a remote CANcoder (raw/sync/fused) for feedback
 * processing inside the IO
 * - Optionally configure one or more follower TalonFX devices
 */
@Builder
public class SubsystemConfig {
  // Naming (for logs/diagnostics)
  @Default
  public final String name = "UNNAMED";

  // Primary TalonFX
  public final int mainId;
  public final String mainBus;

  /** Phoenix configuration to apply to the primary TalonFX. */
  @Default
  public final TalonFXConfiguration fxConfig = new TalonFXConfiguration();

  // Optional remote CANcoder feedback (handled internally by IO when enabled)
  @Default
  public final boolean enableRemoteCANcoder = false;
  public final RemoteCANcoder remoteCANcoder; // nullable unless enableRemoteCANcoder is true

  public final SysidConfig sysidConfig;
  // Optional followers
  @Default
  public final FollowerConfig[] followers = new FollowerConfig[0];

  @Default
  public final InvertedValue motorInvertedValue = InvertedValue.CounterClockwise_Positive;

  @Default
  public final int filterSize = 1;
  @Default
  public final double SensorToMechanismRatio = 1.0;
  /**
   * ONLY used when a linear mechenism meters-per-rotation for linear mechanisms
   * (0.0 disables linear mode).
   */
  @Default
  public final double metersPerRotation = 0.0;

  @Default
  public final boolean updateOutputs = true;

  // other close loop configs
  @Default
  public final GravityTypeValue gravityType = GravityTypeValue.Elevator_Static;
  @Default
  public final StaticFeedforwardSignValue kSValue = StaticFeedforwardSignValue.UseClosedLoopSign;

  /**
   * Soft-limit enables at init; thresholds should be part of fxConfig if used.
   */
  // TODO: soft limit
  @Default
  public final boolean enableForwardSoftLimit = false;
  @Default
  public final boolean enableReverseSoftLimit = false;

  /** Optional remote CANcoder configuration summary. */
  @Builder
  @AllArgsConstructor
  public static class RemoteCANcoder {
    public final int id;
    public final String bus;
    public final double magnetOffset;
    public final double rotorToSensorRatio;
    @Default
    public final SensorDirectionValue sensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    @Default
    public final FeedbackSensorSourceValue feedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    @Default
    public final boolean useContinousWrap = false;

  }

  @Builder
  @AllArgsConstructor
  public static class SysidConfig {
    @Default
    public double sysIdRampRateVoltsPerSec = 1.0;
    @Default
    public double sysIdDynamicVoltage = 5.0;
  }

  /** Optional follower TalonFX configuration. */
  @Builder
  @AllArgsConstructor
  public static class FollowerConfig {
    public final int id;
    public final String bus;

    /** If true, follower output is inverted (oppose main). */
    @Default
    public final boolean opposeMain = false;

  }

  @Builder
  @AllArgsConstructor
  public static class SimConfig {
    @Default
    public final MomentOfInertia MOI = KilogramSquareMeters.of(0);
    @Default
    public final double gearRatio = 1.0d;
    @Default
    public final double[] stdvs = new double[] { 0.0d, 0.0d };
    @Default
    public final TrapezoidProfile.Constraints profile = new TrapezoidProfile.Constraints(50.0, 100.0);
  }

  @Default
  public SimConfig simConfig = new SimConfig(null, 0, null, null);
}
