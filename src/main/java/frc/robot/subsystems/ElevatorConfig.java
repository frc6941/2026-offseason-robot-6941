package frc.robot.subsystems;

import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import lib.ironpulse.subsystem.SubsystemConfig;
import lib.ironpulse.subsystem.servo.ServoParamSources;
import lib.ironpulse.subsystem.SubsystemConfig.SimConfig;
import lib.ntext.NTParameter;

import static edu.wpi.first.units.Units.*;

/** Constants and NT-backed params for {@link ElevatorSubsystem}. */
public final class ElevatorConfig {
  private ElevatorConfig() {}

  public static final String NAME = "Elevator";
  public static final String CANIVORE_CAN_BUS_NAME = "6941Canivore0";

  // Local hardware constants
  private static final int ELEVATOR_MOTOR_MAIN_ID = 50;
  private static final int ELEVATOR_MOTOR_FOLLOWER_ID = 51;
  
  // Physical constants
  private static final double ELEVATOR_SPOOL_DIAMETER = 0.043; // meters (0.04m spool + 0.003m rope)
  private static final double ELEVATOR_GEAR_RATIO = 3.0; // motor rotations per spool rotation
  public static final int ELEVATOR_ZEROING_FILTER_SIZE = 5;
  public static final double METERS_PER_ROTATION = Math.PI * ELEVATOR_SPOOL_DIAMETER;
    public static final SimConfig SIM_CONFIG = SubsystemConfig.SimConfig.builder()
    .MOI(KilogramSquareMeters.of(1))
    .gearRatio(1)
    .stdvs(new double[] {0.02, 0.02})
    .build();
  // Mechanism config
  public static final SubsystemConfig CONFIG = SubsystemConfig.builder()
    .name(NAME)
    .mainId(ELEVATOR_MOTOR_MAIN_ID)
    .mainBus(CANIVORE_CAN_BUS_NAME)
    .motorInvertedValue(InvertedValue.Clockwise_Positive)
    .SensorToMechanismRatio(ELEVATOR_GEAR_RATIO)
    .gravityType(GravityTypeValue.Elevator_Static)
    .zeroingConfig(SubsystemConfig.ZeroingConfig.builder()
        .zeroingCurrentLimit(ElevatorParams.zeroingCurrent)
        .zeroingVoltage(-3.5)
        .zeroingFilterSize(ELEVATOR_ZEROING_FILTER_SIZE)
        .build())
    .followers(new SubsystemConfig.FollowerConfig[] {
      SubsystemConfig.FollowerConfig.builder()
        .bus(CANIVORE_CAN_BUS_NAME)
        .id(ELEVATOR_MOTOR_FOLLOWER_ID)
        .opposeMain(true)
        .build()
    })
    .simConfig(SIM_CONFIG)
    .build();


  @NTParameter(tableName = "Params/" + NAME)
  public static final class ElevatorParams {

    public static final double kP = 3.75;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kV = 0.1308;
    public static final double kA = 0.0068;
    public static final double kS = 0.13;
    public static final double kG = 0.32;

    // Motion Magic - Up (slower, controlled)
    public static final double motionMagicVelRPSUp = 250.0;
    public static final double motionMagicAccelRPS2Up = 600.0;
    public static final double motionMagicJerkRPS3Up = 0.0;
    
    // Motion Magic - Down (faster, gravity-assisted)
    public static final double motionMagicVelRPSDown = 160.0;
    public static final double motionMagicAccelRPS2Down = 200.0;
    public static final double motionMagicJerkRPS3Down = 0.0;

    // Tolerances / behavior
    public static final double atGoalToleranceDegrees = 3.5;
    public static final double atGoalToleranceMeters = 0.03;
    // Zeroing
    public static final double zeroingCurrent = 50.0;
  }
}


