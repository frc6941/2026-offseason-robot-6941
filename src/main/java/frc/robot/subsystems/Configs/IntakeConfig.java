package frc.robot.subsystems.Configs;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.RobotConstants.CANIVORE_CAN_BUS;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.units.measure.Distance;
import lib.ironpulse.subsystem.SubsystemConfig;
import lib.ntext.NTParameter;

public class IntakeConfig {
  public static final String INTAKER_ROLLER_NAME = "IntakerRoller";
  public static final String INTAKER_EXTENSION_NAME = "IntakerExtension";
  public static final Distance INTAKE_EXTENSION_METERS_PER_ROTATION = Meters.of(0.10511);
  private static final int INTAKER_ROLLER_MOTOR_MAIN_ID = 32;
  private static final double INTAKER_ROLLER_GEAR_RATIO = 26.0 / 12.0;
  private static final int INTAKER_EXTENSION_MOTOR_MAIN_ID = 33;
  private static final double INTAKER_EXTENSION_GEAR_RATIO = 26.0 / 40.0 * 15 / 1;
  private static final double INTAKER_ROLLER_STATOR_CURRENT_LIMIT_AMPS = 60;
  private static final double INTAKER_ROLLER_SUPPLY_CURRENT_LIMIT_AMPS = 55;
  public static final SubsystemConfig INTAKER_ROLLER_CONFIG =
      SubsystemConfig.builder()
          .name(INTAKER_ROLLER_NAME)
          .mainBus(CANIVORE_CAN_BUS)
          .mainId(INTAKER_ROLLER_MOTOR_MAIN_ID)
          .motorInvertedValue(InvertedValue.Clockwise_Positive)
          .statorCurrentLimitAmps(INTAKER_ROLLER_STATOR_CURRENT_LIMIT_AMPS)
          .supplyCurrentLimitAmps(INTAKER_ROLLER_SUPPLY_CURRENT_LIMIT_AMPS)
          .defaultBrake(true)
          .kSValue(StaticFeedforwardSignValue.UseVelocitySign)
          .SensorToMechanismRatio(INTAKER_ROLLER_GEAR_RATIO)
          .simConfig(
              SubsystemConfig.SimConfig.builder().gearRatio(INTAKER_ROLLER_GEAR_RATIO).build())
          .build();
  private static final double INTAKER_EXTENSION_STATOR_CURRENT_LIMIT_AMPS = 35;
  private static final double INTAKER_EXTENSION_SUPPLY_CURRENT_LIMIT_AMPS = 20;
  public static final SubsystemConfig INTAKER_EXTENSION_CONFIG =
      SubsystemConfig.builder()
          .name(INTAKER_EXTENSION_NAME)
          .mainBus(CANIVORE_CAN_BUS)
          .mainId(INTAKER_EXTENSION_MOTOR_MAIN_ID)
          .motorInvertedValue(InvertedValue.CounterClockwise_Positive)
          .defaultBrake(true)
          .kSValue(StaticFeedforwardSignValue.UseClosedLoopSign)
          .SensorToMechanismRatio(INTAKER_EXTENSION_GEAR_RATIO)
          .statorCurrentLimitAmps(INTAKER_EXTENSION_STATOR_CURRENT_LIMIT_AMPS)
          .supplyCurrentLimitAmps(INTAKER_EXTENSION_SUPPLY_CURRENT_LIMIT_AMPS)
          .simConfig(
              SubsystemConfig.SimConfig.builder().gearRatio(INTAKER_EXTENSION_GEAR_RATIO).build())
          .zeroingConfig(
              SubsystemConfig.ZeroingConfig.builder()
                  .zeroingCurrentLimit(35)
                  .zeroingFilterSize(5)
                  .zeroingVoltage(-2)
                  .build())
          .build();

  private IntakeConfig() {}

  @NTParameter(tableName = "Params/" + INTAKER_ROLLER_NAME)
  public static final class IntakerRollerParams {
    // velocity gains
    public static final double kP = 1;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kV = 0.1308;
    public static final double kA = 0.0068;
    public static final double kS = 0.13;

    public static final double velocityAtGoalToleranceRPS = 30;

    public static final double testVelRPS = 110;
    public static final double intakeVelRPS = 50;
    public static final double outtakeVelRPS = -50;
    public static final double idleVelRPS = 0;
  }

  @NTParameter(tableName = "Params/" + INTAKER_EXTENSION_NAME)
  public static final class IntakerExtensionParams {

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
    public static final double deployPosMeters = 0.305;
    public static final double feedPosMeters = 0.17;
    public static final double retractPosMeters = 0.01;

    /** Oscillation rate (Hz) for runFeed: deploy <-> feed cycles per second */
    public static final double feedOscillationRateHz = 2;

    public static final boolean isBrake = false;
  }
}
