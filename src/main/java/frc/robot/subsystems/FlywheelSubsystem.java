package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.RobotBase;
import lib.ironpulse.io.MotorIO;
import lib.ironpulse.io.MotorIOSim;
import lib.ironpulse.io.MotorIOTalonFX;
import lib.ironpulse.io.MotorInputsAutoLogged;

/** Flywheel mechanism for velocity control (shooter/intake rollers), with hardware abstraction. */
public class FlywheelSubsystem extends lib.ironpulse.subsystem.FlywheelSubsystem<MotorInputsAutoLogged, MotorIO> {

  public FlywheelSubsystem() {
    super(
      FlywheelConfig.CONFIG,
      new MotorInputsAutoLogged(),
      createIO(),
      FlywheelParamsNT.asFlywheelParamSources()
    );
  }

  private static MotorIO createIO() {
    if (Logger.hasReplaySource()) {
      return new MotorIO() {};
    } else if (RobotBase.isReal()) {
      System.out.println("[Flywheel] Using REAL hardware (TalonFX ID: " + 
        FlywheelConfig.CONFIG.mainId + ")");
      return new MotorIOTalonFX(FlywheelConfig.CONFIG);
    } else {
      System.out.println("[Flywheel] Using SIMULATION");
      return new MotorIOSim(FlywheelConfig.CONFIG);
    }
  }
}


