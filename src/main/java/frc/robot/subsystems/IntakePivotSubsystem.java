package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.RobotBase;
import lib.ironpulse.io.MotorIO;
import lib.ironpulse.io.MotorIOSim;
import lib.ironpulse.io.MotorIOTalonFX;
import lib.ironpulse.io.MotorInputsAutoLogged;
import lib.ironpulse.subsystem.servo.ServoMotorSubsystem;

/**
 * Intake pivot mechanism using a TalonFX and remote CANcoder, extending
 * ServoMotorSubsystem.
 */
public class IntakePivotSubsystem extends ServoMotorSubsystem<MotorInputsAutoLogged, MotorIO, Angle> {

  public IntakePivotSubsystem() {
    super(
        IntakePivotConfig.CONFIG,
        new MotorInputsAutoLogged(),
        createIO(),
        IntakePivotParamsNT.asServoMotorParamSources(),
        Degrees.of(0),
        Degrees.of(360));
  }

  private static MotorIO createIO() {
    if (Logger.hasReplaySource()) {
      return new MotorIO() {
      };
    } else if (RobotBase.isReal()) {
      return new MotorIOTalonFX(IntakePivotConfig.CONFIG);
    } else {
      return new MotorIOSim(IntakePivotConfig.CONFIG);
    }
  }
}
