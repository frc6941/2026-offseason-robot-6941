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
      System.out.println("[IntakePivot] Using REAL hardware (TalonFX ID: " + 
        IntakePivotConfig.CONFIG.mainId + ")");
      return new MotorIOTalonFX(IntakePivotConfig.CONFIG);
    } else {
      // 模拟模式：使用物理模拟
      System.out.println("[IntakePivot] Using SIMULATION");
      return new MotorIOSim(IntakePivotConfig.CONFIG);
    }
  }
}


