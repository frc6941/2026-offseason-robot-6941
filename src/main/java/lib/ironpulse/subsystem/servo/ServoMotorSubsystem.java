package lib.ironpulse.subsystem.servo;

import static edu.wpi.first.units.Units.*;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import lib.ironpulse.subsystem.MotorSubsystem;
import lib.ironpulse.subsystem.SubsystemConfig;
import lib.ironpulse.utils.LoggedTracer;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;

import edu.wpi.first.math.MathUtil;
import lib.ironpulse.io.MotorIO;
import lib.ironpulse.io.MotorInputsAutoLogged;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class ServoMotorSubsystem<T extends MotorInputsAutoLogged, U extends MotorIO> extends MotorSubsystem<T, U> {
  private final LinearFilter currentFilter;
  private final ServoOutputsAutoLogged outputs = new ServoOutputsAutoLogged();
  private boolean zeroing = false;
  private boolean runningCharacterization, reverse = false;

  public double currentFilterValue = 0.0;

  private final SysIdRoutine sysIdRoutine;
  @Getter
  private ServoSetpoint currSetpoint = new ServoSetpoint(ModeServo.VOLTAGE, Degrees.of(0), () -> 0.0);
  private ServoSetpoint tempSetpoint = new ServoSetpoint(ModeServo.VOLTAGE, Degrees.of(0), () -> 0.0);
  @Getter
  private ServoSetpoint prevSetpoint = new ServoSetpoint(ModeServo.VOLTAGE, Degrees.of(0), () -> 0.0);

  private final SubsystemConfig config;
  protected final ServoParamSources params;
  private final Slot0Configs slot0Configs;
  private final MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();

  public ServoMotorSubsystem(SubsystemConfig config, T inputs, U io, ServoParamSources params) {
    super(config, inputs, io);
    this.config = config;
    this.params = params;
    slot0Configs = new Slot0Configs();
    slot0Configs.kP = params.kP();
    slot0Configs.kI = params.kI();
    slot0Configs.kD = params.kD();
    slot0Configs.kA = params.kA();
    slot0Configs.kV = params.kV();
    slot0Configs.kS = params.kS();
    slot0Configs.kG = params.kG();
    io.updateGains(slot0Configs);
    currentFilter = LinearFilter.movingAverage(config.filterSize);

    this.sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            Units.Volts.of(config.sysidConfig.sysIdRampRateVoltsPerSec).per(Units.Second),
            Units.Volts.of(config.sysidConfig.sysIdDynamicVoltage),
            null,
            (state) -> SignalLogger.writeString("sysid-state", state.toString())),
        new SysIdRoutine.Mechanism(
            (Voltage volts) -> {
              currSetpoint = new ServoSetpoint(ModeServo.VOLTAGE, currSetpoint.setPoint, () -> volts.in(Volts));
              SignalLogger.writeDouble("sysid-" + config.name + "-voltage", inputs.appliedVolts, "V");
              SignalLogger.writeDouble("sysid-" + config.name + "-position", getMechanismPositionFromMotor().in(Meters),
                  "m");
              SignalLogger.writeDouble("sysid-" + config.name + "-velocity",
                  getMechanismVelocityFromMotor().in(MetersPerSecond), "m/s");
            },
            null,
            this));
  }

  @Override
  public void periodic() {
    
    Logger.recordOutput("BBB", inputs.positionRot);
    Logger.recordOutput("CCC", inputs.positionRot * config.metersPerRotation);
    Logger.recordOutput("DDD", getMechanismPositionFromMotor());
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

    updateOutputs();
    Logger.recordOutput("AAA", setPointHasChanged());

    if (setPointHasChanged()) {
      switch (currSetpoint.modeServo) {
        case MOTIONMAGIC:
          io.setMotionMagicSetpoint(currSetpoint.setPoint, motionMagicConfigs.MotionMagicCruiseVelocity,
                  motionMagicConfigs.MotionMagicAcceleration, motionMagicConfigs.MotionMagicJerk);
          break;
        case POSITION:
          io.setPositionSetpoint(currSetpoint.setPoint);
          break;
        case DUTY_CYCLE:
          io.setOpenLoopDutyCycle(currSetpoint.openLoop.getAsDouble());
          break;
        case VOLTAGE:
          io.setVoltage(currSetpoint.openLoop.getAsDouble());
          break;
        default:
          break;
      }
    }
    
    Logger.processInputs("Subsystem/" + config.name + "/output", outputs);
    prevSetpoint = new ServoSetpoint(currSetpoint.modeServo, currSetpoint.setPoint, currSetpoint.openLoop);

    if (runningCharacterization) {
      SignalLogger.writeDouble(config.name + "-motor-voltage", inputs.appliedVolts, "V");
      SignalLogger.writeDouble(config.name + "-position", getMechanismPositionFromMotor().in(Meters), "m");
      SignalLogger.writeDouble(config.name + "-velocity", getMechanismVelocityFromMotor().in(MetersPerSecond), "m/s");
      SignalLogger.writeDouble(config.name + "-applied-volts", inputs.appliedVolts, "V");
      SignalLogger.writeDouble(config.name + "-stator-current", inputs.currentStatorAmps, "A");
    }

    LoggedTracer.record(config.name);
  }

  public void updateOutputs() {
    outputs.currentFilterValue = currentFilterValue;
    outputs.setpointDegrees = currSetpoint.setPoint.in(Degrees);
    outputs.positionAtGoal = positionAtGoal();
    outputs.openLoopValue = currSetpoint.openLoop.getAsDouble();
    outputs.mechanismPositionMeters = getMechanismPositionFromMotor().in(Meters);
    outputs.mechanismVelocityMps = getMechanismVelocityFromMotor().in(MetersPerSecond);
    outputs.isRunningCharacterization = runningCharacterization;
    outputs.isZeroing = zeroing;
    switch (currSetpoint.modeServo) {
      case MOTIONMAGIC:
        outputs.currentMode = "MOTION_MAGIC";
        break;
      case POSITION:
        outputs.currentMode = "POSITION";
        break;
      case DUTY_CYCLE:
        outputs.currentMode = "DUTY_CYCLE";
        break;
      case VOLTAGE:
        outputs.currentMode = "VOLTAGE";
        break;
      default:
        outputs.currentMode = "";
        break;
    }
  }

  public boolean positionAtGoal(Angle tolerance) {
    return Rotations.of(inputs.positionRot).isNear(currSetpoint.setPoint, tolerance);
  }

  public boolean positionAtGoal() {
    return positionAtGoal(Degrees.of(params.positionAtGoalToleranceDegrees()));
  }

  public void setCurrentPositionAsZero() {
    io.setCurrentPositionAsZero();
  }

  public void setMotionMagicSetpoint(Angle position) {
    motionMagicConfigs.MotionMagicAcceleration = params.motionMagicAccelRPS2();
    motionMagicConfigs.MotionMagicCruiseVelocity = params.motionMagicVelRPS();
    motionMagicConfigs.MotionMagicJerk = params.motionMagicJerkRPS3();
    double accel = params.motionMagicAccelRPS2();
    double jerk = params.motionMagicJerkRPS3();
    currSetpoint.modeServo = ModeServo.MOTIONMAGIC;
    currSetpoint.setPoint = position;
  }

  public void setOpenLoopDutyCycle(double dutyCycle) {
    currSetpoint.openLoop = () -> MathUtil.clamp(dutyCycle, -1.0d, 1.0d);
    currSetpoint.modeServo = ModeServo.DUTY_CYCLE;
  }

  public void setVoltage(double voltage) {
    currSetpoint.openLoop = () -> MathUtil.clamp(voltage, -12.0d, 12.0d);
    currSetpoint.modeServo = ModeServo.VOLTAGE;
  }

  public void setMotionMagicSetpoint(Angle position, double velocity, double acceleration, double jerk) {
    currSetpoint.modeServo = ModeServo.MOTIONMAGIC;
    currSetpoint.setPoint = position;
    motionMagicConfigs.MotionMagicAcceleration = acceleration;
    motionMagicConfigs.MotionMagicCruiseVelocity = velocity;
    motionMagicConfigs.MotionMagicJerk = jerk;
  }

  public void zeroSubsystem(boolean isReverse) {

          zeroing = true;
          tempSetpoint = new ServoSetpoint(currSetpoint.modeServo, currSetpoint.setPoint, currSetpoint.openLoop);

          if (RobotBase.isReal()) {
            currentFilterValue = currentFilter.calculate(inputs.currentStatorAmps);
            if (currentFilterValue <= params.zeroingCurrentLimit()) {
              currSetpoint.openLoop = isReverse ? () -> 1 : () -> -1;
              currSetpoint.modeServo = ModeServo.VOLTAGE;
            }
            if (currentFilterValue > params.zeroingCurrentLimit()) {
              currSetpoint.openLoop = () -> 0;
              currSetpoint.modeServo = ModeServo.VOLTAGE;
              setCurrentPositionAsZero();
              zeroing = false;
            }
          } else {
            setMotionMagicSetpoint(Rotations.of(0));
            if (Math.abs(inputs.positionRot) < 0.01) {
              zeroing = false;
            }
          }


          zeroing = false;
          currSetpoint = tempSetpoint;

  }

  // SysId characterization commands
  /**
   * Returns a command that runs a quasistatic test in the given direction.
   *
   * @param direction The direction to run the test (kForward = up, kReverse =
   *                  down)
   * @return The SysId quasistatic command
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return Commands.sequence(
        Commands.runOnce(() -> tempSetpoint = new ServoSetpoint(currSetpoint.modeServo, currSetpoint.setPoint,
            currSetpoint.openLoop)),
        Commands.runOnce(() -> runningCharacterization = true),
        sysIdRoutine.quasistatic(direction),
        Commands.runOnce(() -> runningCharacterization = false),
        Commands.runOnce(() -> currSetpoint = tempSetpoint));
  }

  /**
   * Returns a command that runs a dynamic test in the given direction.
   *
   * @param direction The direction to run the test (kForward = up, kReverse =
   *                  down)
   * @return The SysId dynamic command
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return Commands.sequence(
        Commands.runOnce(() -> tempSetpoint = new ServoSetpoint(currSetpoint.modeServo, currSetpoint.setPoint,
            currSetpoint.openLoop)),
        Commands.runOnce(() -> runningCharacterization = true),
        sysIdRoutine.dynamic(direction),
        Commands.runOnce(() -> runningCharacterization = false),
        Commands.runOnce(() -> currSetpoint = tempSetpoint));
  }

  /**
   * Returns a command that runs the complete SysId characterization sequence.
   * Automatically starts SignalLogger, pauses climber, runs all 4 tests, then
   * stops logging.
   *
   * @return Complete SysId characterization command sequence
   */
  public Command sysIdComplete() {
    return Commands.sequence(
        Commands.runOnce(SignalLogger::start),
        Commands.print("Starting " + config.name + " SysId - Climber Paused"),
        Commands.waitSeconds(0.5), // Let climber settle
        Commands.print("Starting " + config.name + " SysId - Quasistatic Forward"),
        sysIdQuasistatic(SysIdRoutine.Direction.kForward),
        Commands.waitSeconds(1.0), // Brief pause between tests
        Commands.print("Starting " + config.name + " SysId - Quasistatic Reverse"),
        sysIdQuasistatic(SysIdRoutine.Direction.kReverse),
        Commands.waitSeconds(1.0),
        Commands.print("Starting " + config.name + " SysId - Dynamic Forward"),
        sysIdDynamic(SysIdRoutine.Direction.kForward),
        Commands.waitSeconds(1.0),
        Commands.print("Starting " + config.name + " SysId - Dynamic Reverse"),
        sysIdDynamic(SysIdRoutine.Direction.kReverse),
        Commands.runOnce(SignalLogger::stop),
        Commands.print(config.name + " SysId Complete - Check logs"));
  }

  public void setPositionSetpoint(Angle position) {
    currSetpoint.modeServo = ModeServo.POSITION;
    currSetpoint.setPoint = position;
  }

  public Distance getMechanismPositionFromMotor() {
    double position = inputs.positionRot * config.metersPerRotation;
    return Meters.of(reverse ? -position : position);
  }

  public LinearVelocity getMechanismVelocityFromMotor() {
    double velocity = inputs.velocityRotPerSecond * config.metersPerRotation;
    return MetersPerSecond.of(reverse ? -velocity : velocity);
  }

  private enum ModeServo {
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

  public static class ServoSetpoint {
    private ModeServo modeServo;
    private Angle setPoint;
    private DoubleSupplier openLoop;

    private ServoSetpoint(ModeServo modeServo, Angle setPoint, DoubleSupplier openLoop) {
      this.modeServo = modeServo;
      this.setPoint = setPoint;
      this.openLoop = openLoop;
    }
  }
}
