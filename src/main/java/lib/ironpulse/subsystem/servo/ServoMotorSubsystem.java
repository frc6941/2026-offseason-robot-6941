package lib.ironpulse.subsystem.servo;

import static edu.wpi.first.units.Units.*;

import java.util.function.DoubleSupplier;

import edu.wpi.first.units.measure.*;
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

public class ServoMotorSubsystem<T extends MotorInputsAutoLogged, U extends MotorIO> extends MotorSubsystem<T, U> {
  private final ServoOutputsAutoLogged outputs = new ServoOutputsAutoLogged();
  @Getter
  @AutoLogOutput(key = "ServoMotorSubsystem/currSetpoint")
  private ServoSetpoint currSetpoint = new ServoSetpoint(ModeServo.VOLTAGE, Degrees.of(0), () -> 0.0);
  @Getter
  private ServoSetpoint prevSetpoint = new ServoSetpoint(ModeServo.VOLTAGE, Degrees.of(0), () -> 0.0);

  private final SubsystemConfig config;
  protected final ServoParamSources params;
  private final Angle zeroOffset;
  private final Slot0Configs slot0Configs;
  private final MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();

  public ServoMotorSubsystem(SubsystemConfig config, T inputs, U io, ServoParamSources params) {
    super(config, inputs, io);
    this.config = config;
    this.params = params;
    this.zeroOffset = config.zeroOffset;
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

  @Override
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
      switch (currSetpoint.modeServo) {
        case MOTIONMAGIC:
          io.setMotionMagicSetpoint(currSetpoint.setPoint.minus(zeroOffset), motionMagicConfigs.MotionMagicCruiseVelocity,
              motionMagicConfigs.MotionMagicAcceleration, motionMagicConfigs.MotionMagicJerk);
          break;
        case POSITION:
          io.setPositionSetpoint(currSetpoint.setPoint.minus(zeroOffset));
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

    LoggedTracer.record(config.name);
  }

  public boolean positionAtGoal(Angle tolerance) {
    return Rotations.of(getServoAngleRot()).isNear(currSetpoint.setPoint, tolerance);
  }

  public boolean positionAtGoal() {
    return positionAtGoal(Degrees.of(params.positionAtGoalToleranceDegrees()));
  }

  public void setMotionMagicSetpoint(Angle angleDeg) {
    motionMagicConfigs.MotionMagicAcceleration = params.motionMagicAccelRPS2();
    motionMagicConfigs.MotionMagicCruiseVelocity = params.motionMagicVelRPS();
    motionMagicConfigs.MotionMagicJerk = params.motionMagicJerkRPS3();
    currSetpoint.modeServo = ModeServo.MOTIONMAGIC;
    currSetpoint.setPoint = angleDeg;
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

  public void setPositionSetpoint(Angle position) {
    currSetpoint.modeServo = ModeServo.POSITION;
    currSetpoint.setPoint = position;
  }

  public double getServoAngleRot() {
    return inputs.positionRot + zeroOffset.in(Rotations);
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
