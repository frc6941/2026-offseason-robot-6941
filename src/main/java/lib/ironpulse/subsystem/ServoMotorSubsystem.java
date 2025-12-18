package lib.ironpulse.subsystem;

import static edu.wpi.first.units.Units.*;

import java.lang.StackWalker.Option;
import java.util.Optional;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.Slot0Configs;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import lib.ironpulse.io.MotorIO;
import lib.ironpulse.io.MotorInputsAutoLogged;
import lib.ironpulse.swerve.Swerve.MODE;
import lombok.Getter;

public class ServoMotorSubsystem<T extends MotorInputsAutoLogged, U extends MotorIO> extends MotorSubsystem<T, U> {

  @Getter
  private ServoSetpoint currSetpoint = new ServoSetpoint(ModeServo.VOLTAGE, Degrees.of(0), () -> 0.0);
  @Getter
  private ServoSetpoint prevSetpoint = new ServoSetpoint(ModeServo.VOLTAGE, Degrees.of(0), () -> 0.0);

  private final SubsystemConfig config;
  protected final ParamSources params;
  private final Slot0Configs slot0Configs;
  private ModeServo currentMode = ModeServo.VOLTAGE;

  public ServoMotorSubsystem(SubsystemConfig config, T inputs, U io, ParamSources params) {
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
  }

  @Override
  public void periodic() {
    Logger.recordOutput(config.name, currSetpoint.modeServo);
    Logger.recordOutput(config.name, currSetpoint.setpt.in(Degrees));
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

    if (setptHasChanged()) {
      switch (currSetpoint.modeServo) {
        case MOTIONMAGIC:
          io.setMotionMagicSetpoint(currSetpoint.setpt, params.motionMagicVelRPS(), params.motionMagicAccelRPS2(),
              params.motionMagicJerkRPS3());
          break;
        case POSITION:
          io.setPositionSetpoint(currSetpoint.setpt);
          break;
        case DUTY_CYCLE:
          runDutyCycle(currSetpoint.openLoop);
          break;
        case VOLTAGE:
          runVoltage(currSetpoint.openLoop);
          break;
        default:
          break;
      }
    }
    prevSetpoint = currSetpoint;

  }

  public boolean positionAtGoal(Angle tolerance) {
    return Rotations.of(inputs.positionRot).isNear(currSetpoint.setpt, tolerance);
  }

  public boolean positionAtGoal() {
    return positionAtGoal(Degrees.of(params.positionAtGoalToleranceDegrees()));
  }

  public void setCurrentPositionAsZero() {
    io.setCurrentPositionAsZero();
  }

  public void setMotionMagicSetpoint(Angle position) {
    double vel = params.motionMagicVelRPS();
    double accel = params.motionMagicAccelRPS2();
    double jerk = params.motionMagicJerkRPS3();
    currSetpoint.modeServo = ModeServo.MOTIONMAGIC;
    currSetpoint.setpt = position;
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
    currSetpoint.setpt = position;
    currentMode = ModeServo.MOTIONMAGIC;
  }

  public void setPositionSetpoint(Angle position) {
    currSetpoint.modeServo = ModeServo.POSITION;
    currSetpoint.setpt = position;
  }

  // hack to hook NTParameterProcessor to generate ParamSources for uses in
  // subsystems
  // REMEMBER to update NTParameterProcessor when adding new fields to
  // ParamSources
  public interface ParamSources {
    double kP();

    double kI();

    double kD();

    default double kA() {
      return 0.0;
    }

    default double kV() {
      return 0.0;
    }

    default double kS() {
      return 0.0;
    }

    default double kG() {
      return 0.0;
    }

    default double motionMagicVelRPS() {
      return 0.0;
    }

    default double motionMagicAccelRPS2() {
      return 0.0;
    }

    default double motionMagicJerkRPS3() {
      return 0.0;
    }

    default double positionAtGoalToleranceDegrees() {
      return 1.0;
    }

    default double positionAtGoalToleranceMeters() {
      return 0.005;
    }

    default boolean isBrake() {
      return true;
    }

    /* hook to ParamsNT.isAnyChanged() */
    default boolean hasChanged() {
      return false;
    }
  }

  private enum ModeServo {
    POSITION,
    MOTIONMAGIC,
    VOLTAGE,
    DUTY_CYCLE
  }

  private boolean setptHasChanged() {
    return !(currSetpoint.setpt == prevSetpoint.setpt) && currSetpoint.modeServo == prevSetpoint.modeServo;
  }

  public static class ServoSetpoint {
    private ModeServo modeServo;
    private Angle setpt;
    private DoubleSupplier openLoop;

    private ServoSetpoint(ModeServo modeServo, Angle setpt, DoubleSupplier openLoop) {
      this.modeServo = modeServo;
      this.setpt = setpt;
      this.openLoop = openLoop;
    }
  }
}
