package frc.robot.subsystems;

import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import lib.ironpulse.subsystem.SubsystemConfig;
import lib.ironpulse.subsystem.ServoMotorSubsystem.ParamSources;
import lib.ironpulse.subsystem.SubsystemConfig.SimConfig;
import lib.ntext.NTParameter;

import static edu.wpi.first.units.Units.*;

/** Constants and NT-backed params for {@link IntakePivotSubsystem}. */
public final class IntakePivotConfig {
  private IntakePivotConfig() {}

  public static final String NAME = "IntakePivot";
  public static final String CANIVORE_CAN_BUS_NAME = "10541Canivore0";

  // Local hardware constants (copied/adapted from provided snippet)
  private static final int INTAKE_PIVOT_MOTOR_ID = 33;
  // private static final int INTAKE_PIVOT_ENCODER_ID = 17;
  // private static final double INTAKE_PIVOT_ROTOR_ENCODER_RATIO = 80.1818181818; // (45/11)*(56/20)*(56/8)
  // private static final double INTAKE_PIVOT_ENCODER_OFFSET = -0.382568; // -0.132568 - 0.25

  // Mechanism config
  public static final SubsystemConfig CONFIG = SubsystemConfig.builder()
    .name(NAME)
    .mainId(INTAKE_PIVOT_MOTOR_ID)
    .mainBus(CANIVORE_CAN_BUS_NAME)
    .motorInvertedValue(InvertedValue.Clockwise_Positive)
    .SensorToMechanismRatio(1.0)
    .gravityType(GravityTypeValue.Arm_Cosine)
    // .enableRemoteCANcoder(true)
    // .remoteCANcoder(SubsystemConfig.RemoteCANcoder.builder()
    //   .id(INTAKE_PIVOT_ENCODER_ID)
    //   .bus(CANIVORE_CAN_BUS_NAME)
    //   .magnetOffset(INTAKE_PIVOT_ENCODER_OFFSET)
    //   .rotorToSensorRatio(INTAKE_PIVOT_ROTOR_ENCODER_RATIO)
    //   .sensorDirection(SensorDirectionValue.CounterClockwise_Positive)
    //   .feedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
    //   .useContinousWrap(false)
    //   .build())
    .build();

    public static final SimConfig SIM_CONFIG = SubsystemConfig.SimConfig.builder()
    .MOI(KilogramSquareMeters.of(0.04))
    .gearRatio(1)
    .stdvs(new double[] {0.02, 0.02})
    .build();

  @NTParameter(tableName = "Params/" + NAME)
  public static final class IntakePivotParams {
    // PID/FF gains
    public static final double kP = 4.7;
    public static final double kI = 0.5;
    public static final double kD = 0.02;
    public static final double kV = 0.12;
    public static final double kA = 0.11;
    public static final double kS = 0.0;
    public static final double kG = -0.035;

    public static final double motionMagicVelRPS = 100.0;       
    public static final double motionMagicAccelRPS2 = 100.0;
    public static final double motionMagicJerkRPS3 = 1000.0;   

    // Tolerances / behavior
    public static final double atGoalToleranceDegrees = 3.5;
    public static final boolean isBrake = false;
  }
}


