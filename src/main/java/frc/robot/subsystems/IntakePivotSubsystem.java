package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.RobotBase;
import lib.ironpulse.io.MotorIO;
import lib.ironpulse.io.MotorIOSim;
import lib.ironpulse.io.MotorIOTalonFX;
import lib.ironpulse.io.MotorInputsAutoLogged;
import lib.ironpulse.subsystem.ServoMotorSubsystem;

/** Intake pivot mechanism using a TalonFX and remote CANcoder, extending ServoMotorSubsystem. */
public class IntakePivotSubsystem extends ServoMotorSubsystem<MotorInputsAutoLogged, MotorIO> {

  public IntakePivotSubsystem() {
    super(
      IntakePivotConfig.CONFIG,
      new MotorInputsAutoLogged(),
      createIO(),
      IntakePivotParamsNT.asServoMotorParamSources()
    );
  }

  private static MotorIO createIO() {
    if (Logger.hasReplaySource()) {
      return new MotorIO() {};
    } else if (RobotBase.isReal()) {
      System.out.println("[IntakePivot] Using REAL hardware (TalonFX ID: " + 
        IntakePivotConfig.CONFIG.mainId + ")");
      return new MotorIOTalonFX(IntakePivotConfig.CONFIG);
    } else {
      System.out.println("[IntakePivot] Using SIMULATION");
      return new MotorIOSim(IntakePivotConfig.CONFIG);
    }
  }
}


