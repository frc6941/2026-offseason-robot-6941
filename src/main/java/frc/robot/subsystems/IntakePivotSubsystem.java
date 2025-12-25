package frc.robot.subsystems;

import lib.ironpulse.subsystem.servo.ServoOutputsAutoLogged;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.RobotBase;
import lib.ironpulse.io.MotorIO;
import lib.ironpulse.io.MotorIOSim;
import lib.ironpulse.io.MotorIOTalonFX;
import lib.ironpulse.io.MotorInputsAutoLogged;
import lib.ironpulse.subsystem.servo.ServoMotorSubsystem;

/** Intake pivot mechanism using a TalonFX and remote CANcoder, extending ServoMotorSubsystem. */
public class IntakePivotSubsystem extends ServoMotorSubsystem<MotorInputsAutoLogged, MotorIO, ServoOutputsAutoLogged> {

  public IntakePivotSubsystem() {
    super(
      IntakePivotConfig.CONFIG,
      new MotorInputsAutoLogged(),
      createIO(),
      new ServoOutputsAutoLogged(),
      IntakePivotParamsNT.asServoMotorParamSources()
    );
  }

  private static MotorIO createIO() {
    if (Logger.hasReplaySource()) {
      return new MotorIO() {};
    } else if (RobotBase.isReal()) {
      return new MotorIOTalonFX(IntakePivotConfig.CONFIG);
    } else {
      System.out.println("[IntakePivot] Using SIMULATION");
      return new MotorIOSim(IntakePivotConfig.CONFIG);
    }
  }
}


