package lib.ironpulse.subsystem.servo;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.DoubleSupplier;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.measure.Angle;
import lib.ironpulse.subsystem.MotorSubsystem;
import lib.ironpulse.subsystem.SubsystemConfig;
import lib.ironpulse.utils.LoggedTracer;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;

import edu.wpi.first.math.MathUtil;
import lib.ironpulse.io.MotorIO;
import lib.ironpulse.io.MotorInputsAutoLogged;
import lombok.Getter;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ServoMotorSubsystem<T extends MotorInputsAutoLogged, U extends MotorIO, M extends Measure<?>> extends MotorSubsystem<T, U> {
  @Getter
  private ServoSetpoint<M> currSetpoint;
  @Getter
  private ServoSetpoint<M> prevSetpoint;
  private boolean zeroing = false;
  private LinearFilter currentFilter = LinearFilter.movingAverage(5);
  private double currentFilterValue = 0.0;

  private final SubsystemConfig config;
  protected final ServoParamSources params;
  private final Angle zeroOffset;
  private final Slot0Configs slot0Configs;
  private final MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
  private final M mechanismUnitPerRotation;

  /**
   * Creates a new generic ServoMotorSubsystem.
   * 
   * @param config The subsystem configuration (IDs, buses, ratios).
   * @param inputs The autologged inputs for the mechanism.
   * @param io The IO interface for the hardware.
   * @param params The parameter sources (PID, MotionMagic limits).
   * @param initialSetpoint The starting position in mechanism units (M).
   * @param mechanismUnitPerRotation The amount of mechanism movement per ONE motor rotation.
   *                                 Example (Elevator): Meters.of(ElevatorConfig.METERS_PER_ROTATION)
   *                                 Example (Pivot): Degrees.of(360)
   */
  public ServoMotorSubsystem(SubsystemConfig config, T inputs, U io, ServoParamSources params, M initialSetpoint, M mechanismUnitPerRotation) {
    super(config, inputs, io);
    this.config = config;
    this.params = params;
    this.zeroOffset = config.zeroOffset;
    this.mechanismUnitPerRotation = mechanismUnitPerRotation;
    this.currSetpoint = new ServoSetpoint<>(SetptTyp.VOLTAGE, initialSetpoint, () -> 0.0);
    this.prevSetpoint = new ServoSetpoint<>(SetptTyp.VOLTAGE, initialSetpoint, () -> 0.0);
    slot0Configs = new Slot0Configs();
    slot0Configs.kP = params.kP();
    slot0Configs.kI = params.kI();
    slot0Configs.kD = params.kD();
    slot0Configs.kA = params.kA();
    slot0Configs.kV = params.kV();
    slot0Configs.kS = params.kS();
    slot0Configs.kG = params.kG();
    io.updateGains(slot0Configs);
  }

  @SuppressWarnings("unchecked")
  private Angle toAngle(M mechanismValue) {
    Unit mechanismUnit = (Unit) mechanismUnitPerRotation.unit();
    return Rotations.of(((Measure) mechanismValue).in(mechanismUnit) / ((Measure) mechanismUnitPerRotation).in(mechanismUnit));
  }

  @SuppressWarnings("unchecked")
  private M fromAngle(Angle motorAngle) {
    return (M) mechanismUnitPerRotation.times(motorAngle.in(Rotations));
  }

  @Override
  @SuppressWarnings("unchecked")
  public void periodic() {

    super.periodic();
    if (params.hasChanged()) {
      this.slot0Configs.kP = params.kP();
      this.slot0Configs.kI = params.kI();
      this.slot0Configs.kD = params.kD();
      this.slot0Configs.kA = params.kA();
      this.slot0Configs.kV = params.kV();
      this.slot0Configs.kS = params.kS();
      this.slot0Configs.kG = params.kG();
      io.setNeutralMode(params.isBrake());
      io.updateGains(slot0Configs);
    }


    if (setPointHasChanged()) {
      Logger.recordOutput(config.name + "/currSetpointType", currSetpoint.getModeServo());
      Unit mechanismUnit = (Unit) mechanismUnitPerRotation.unit();
      switch (currSetpoint.modeServo) {
        case MOTIONMAGIC:
          Logger.recordOutput(config.name + "/currSetpoint", ((Measure) currSetpoint.setPoint).in(mechanismUnit));
          io.setMotionMagicSetpoint(toAngle(currSetpoint.setPoint).minus(zeroOffset), motionMagicConfigs.MotionMagicCruiseVelocity,
              motionMagicConfigs.MotionMagicAcceleration, motionMagicConfigs.MotionMagicJerk);
          break;
        case POSITION:
          Logger.recordOutput(config.name + "/currSetpoint", ((Measure) currSetpoint.setPoint).in(mechanismUnit));
          io.setPositionSetpoint(toAngle(currSetpoint.setPoint).minus(zeroOffset));
          break;
        case DUTY_CYCLE:
          Logger.recordOutput(config.name + "/currSetpoint", currSetpoint.openLoop.getAsDouble());
          io.setOpenLoopDutyCycle(currSetpoint.openLoop.getAsDouble());
          break;
        case VOLTAGE:
          Logger.recordOutput(config.name + "/currSetpoint", currSetpoint.openLoop.getAsDouble());
          io.setVoltage(currSetpoint.openLoop.getAsDouble());
          break;
        default:
          break;
      }
    }

    Logger.recordOutput(config.name + "/atGoal", positionAtGoal());
    Logger.recordOutput(config.name + "/currPosition", ((Measure)getCurrPos()).in((Unit) mechanismUnitPerRotation.unit()));

    prevSetpoint = new ServoSetpoint<>(currSetpoint.modeServo, currSetpoint.setPoint, currSetpoint.openLoop);

    LoggedTracer.record(config.name);
  }

  @SuppressWarnings("unchecked")
  public boolean positionAtGoal(M tolerance) {
    // Cast to raw Measure to bypass generic issues with isNear in some WPILib versions
    return ((Measure) getCurrPos()).isNear((Measure) currSetpoint.setPoint, (Measure) tolerance);
  }

  @SuppressWarnings("unchecked")
  public boolean positionAtGoal() {
    try {
        return positionAtGoal((M) Degrees.of(params.positionAtGoalToleranceDegrees()));
    } catch (ClassCastException e) {
        return positionAtGoal((M) Meters.of(params.positionAtGoalToleranceMeters()));
    }
  }

  public void setMotionMagicSetpoint(M position) {
    motionMagicConfigs.MotionMagicAcceleration = params.motionMagicAccelRPS2();
    motionMagicConfigs.MotionMagicCruiseVelocity = params.motionMagicVelRPS();
    motionMagicConfigs.MotionMagicJerk = params.motionMagicJerkRPS3();
    currSetpoint.modeServo = SetptTyp.MOTIONMAGIC;
    currSetpoint.setPoint = position;
  }

  public void setOpenLoopDutyCycle(double dutyCycle) {
    currSetpoint.openLoop = () -> MathUtil.clamp(dutyCycle, -1.0d, 1.0d);
    currSetpoint.modeServo = SetptTyp.DUTY_CYCLE;
  }

  public void setVoltage(double voltage) {
    currSetpoint.openLoop = () -> MathUtil.clamp(voltage, -12.0d, 12.0d);
    currSetpoint.modeServo = SetptTyp.VOLTAGE;
  }

  public void setMotionMagicSetpoint(M position, double velocity, double acceleration, double jerk) {
    currSetpoint.modeServo = SetptTyp.MOTIONMAGIC;
    currSetpoint.setPoint = position;
    motionMagicConfigs.MotionMagicAcceleration = acceleration;
    motionMagicConfigs.MotionMagicCruiseVelocity = velocity;
    motionMagicConfigs.MotionMagicJerk = jerk;
  }

  public void setPositionSetpoint(M position) {
    currSetpoint.modeServo = SetptTyp.POSITION;
    currSetpoint.setPoint = position;
  }

  /**
   * Returns a command that zeroes the mechanism by driving it until a current spike is detected.
   *
   * @return The zeroing command.
   */
  public Command zeroCommand() {
    Command zeroCommand = Commands.startRun(
            () -> {
              zeroing = true;
              currentFilter = LinearFilter.movingAverage(config.zeroingConfig.zeroingFilterSize);
            },
            () -> {
              if (RobotBase.isReal()) {
                currentFilterValue = currentFilter.calculate(inputs.currentStatorAmps);
                if (currentFilterValue <= config.zeroingConfig.zeroingCurrentLimit) {
                  setVoltage(config.zeroingConfig.zeroingVoltage);
                } else {
                  setVoltage(0);
                  io.setCurrentPositionAsZero();
                  zeroing = false;
                }
              } else {
                // In simulation, just set target to 0 (going down)
                setMotionMagicSetpoint(fromAngle(Rotations.of(0)));
                if (Math.abs(inputs.positionRot) < 0.01) {
                  zeroing = false;
                }
              }
            },
            this)
        .until(() -> !zeroing)
        .finallyDo(
            () -> {
              zeroing = false;
              setVoltage(0);
            });
    zeroCommand.addRequirements(this);
    return zeroCommand;
  }

  public M getCurrPos() {
    return fromAngle(Rotations.of(inputs.positionRot + zeroOffset.in(Rotations)));
  }

  protected enum SetptTyp {
    POSITION,
    MOTIONMAGIC,
    VOLTAGE,
    DUTY_CYCLE
  }

  private boolean setPointHasChanged() {
    return currSetpoint.modeServo != prevSetpoint.modeServo ||
        !currSetpoint.setPoint.equals(prevSetpoint.setPoint) ||
        currSetpoint.openLoop != prevSetpoint.openLoop;
  }

  @Getter
  public static class ServoSetpoint<M extends Measure<?>> {
    private SetptTyp modeServo;
    private M setPoint;
    private DoubleSupplier openLoop;

    private ServoSetpoint(SetptTyp modeServo, M setPoint, DoubleSupplier openLoop) {
      this.modeServo = modeServo;
      this.setPoint = setPoint;
      this.openLoop = openLoop;
    }
  }
}
