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

  /**
   * 根据运行模式创建正确的IO实现
   * - 真实机器人 -> TalonFX硬件
   * - 模拟 -> 物理模拟
   * - Replay -> 空IO（仅用于日志回放）
   */
  private static MotorIO createIO() {
    if (Logger.hasReplaySource()) {
      // Replay模式：使用空IO，所有数据来自日志
      return new MotorIO() {};
    } else if (RobotBase.isReal()) {
      // 真实机器人：使用TalonFX硬件
      System.out.println("[Flywheel] Using REAL hardware (TalonFX ID: " + 
        FlywheelConfig.CONFIG.mainId + ")");
      return new MotorIOTalonFX(FlywheelConfig.CONFIG);
    } else {
      // 模拟模式：使用物理模拟
      System.out.println("[Flywheel] Using SIMULATION");
      return new MotorIOSim(FlywheelConfig.SIM_CONFIG);
    }
  }
}


