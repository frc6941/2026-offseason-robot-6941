package frc.robot.subsystems;

import lib.ironpulse.io.MotorIOSim;
import lib.ironpulse.io.MotorIOTalonFX;
import lib.ironpulse.io.MotorInputsAutoLogged;
import lib.ironpulse.subsystem.ServoMotorSubsystem;

/** Intake pivot mechanism using a TalonFX and remote CANcoder, extending ServoMotorSubsystem. */
public class IntakePivotSubsystem extends ServoMotorSubsystem<MotorInputsAutoLogged, MotorIOTalonFX> {

  public IntakePivotSubsystem() {
    super(
      IntakePivotConfig.CONFIG,
      new MotorInputsAutoLogged(),
      new MotorIOTalonFX(IntakePivotConfig.CONFIG),
      IntakePivotParamsNT.asServoMotorParamSources()
    );
  }
}


