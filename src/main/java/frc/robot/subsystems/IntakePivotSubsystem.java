package frc.robot.subsystems;

import lib.ironpulse.io.MotorIOTalonFX;
import lib.ironpulse.io.MotorInputsAutoLogged;
import lib.ironpulse.subsystem.ServoMotorSubsystem;

/** Intake pivot mechanism using a TalonFX and remote CANcoder, extending ServoMotorSubsystem. */
public class IntakePivotSubsystem extends ServoMotorSubsystem<MotorInputsAutoLogged, MotorIOTalonFX> {

  public IntakePivotSubsystem() {
    super(
      IntakePivotSubsystemConstants.CONFIG,
      new MotorInputsAutoLogged(),
      new MotorIOTalonFX(IntakePivotSubsystemConstants.CONFIG),
      IntakePivotSubsystemConstants.PARAMS
    );
  }
}


