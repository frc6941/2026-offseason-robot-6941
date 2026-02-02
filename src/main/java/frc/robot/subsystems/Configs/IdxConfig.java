package frc.robot.subsystems.Configs;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import frc.robot.RobotConstants;
import lib.ironpulse.subsystem.SubsystemConfig;
import lib.ntext.NTParameter;

public class IdxConfig {
  public static final String SPIN = "Spindexer_Spin";
  public static final String INDEXER = "Spindexer_Indexer";
  public static final String SPINDEXER = "Spindexer";
  public static final String CANIVORE_CAN_BUS_NAME = RobotConstants.CANIVORE_CAN_BUS_NAME;
  public static final double SPINDEXER_GEAR_RATIO = 1.0; // THIS LINE SHOULD NOT BE CHANGED
  public static final double SPIN_GEAR_RATIO = 27.0 / 1.0 * 66.0 / 28.0;
  public static final double INDEXER_GEAR_RATIO =
      27.0 / 1.0 * 66.0 / 28.0 / 86.0 * 12.0 / 28.0 * 22.0 / 22.0 * 15.0;
  private static final int SPINDEXER_ID = 1;
  public static final SubsystemConfig SPINDEXER_CFG =
      SubsystemConfig.builder()
          .name(SPINDEXER)
          .mainBus(CANIVORE_CAN_BUS_NAME)
          .mainId(SPINDEXER_ID)
          .motorInvertedValue(InvertedValue.Clockwise_Positive)
          .defaultBrake(true)
          .kSValue(StaticFeedforwardSignValue.UseVelocitySign)
          .SensorToMechanismRatio(SPINDEXER_GEAR_RATIO)
          .simConfig(SubsystemConfig.SimConfig.builder().gearRatio(SPINDEXER_GEAR_RATIO).build())
          .build();

  private IdxConfig() {}

  /** Target speed in RPS */
  @NTParameter(tableName = "Params/IdxModes")
  public static final class IdxModeParams {
    public static final double feedSpindexer = 110.0;

    public static final double revSpindexer = -110.0;
  }

  @NTParameter(tableName = "Params/" + SPINDEXER)
  public static final class SpindexerParams {
    public static final double kP = 1;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kV = 0.1308;
    public static final double kA = 0.0068;
    public static final double kS = 0.13;

    public static final double velocityAtGoalToleranceRPS = 30;
  }
}
