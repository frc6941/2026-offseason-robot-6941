package frc.robot.subsystems.ShootingSubsystem;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.subsystems.ShootingSubsystem.SpindexerSubsystem.IdxDevice.*;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Configs.IdxConfig;
import frc.robot.subsystems.Configs.IdxModeParamsNT;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;
import lib.ironpulse.io.MotorIO;
import lib.ironpulse.io.MotorInputsAutoLogged;
import lib.ironpulse.subsystem.velocity.VelocityMotorSubsystem;
import org.littletonrobotics.junction.Logger;

public class SpindexerSubsystem {
  private final VelocityMotorSubsystem<MotorInputsAutoLogged, MotorIO> spindexer;
  Map<IdxDevice, Double> GEAR_RATIO_DICT = new HashMap<>();

  public SpindexerSubsystem(
      VelocityMotorSubsystem<MotorInputsAutoLogged, MotorIO> spindexer,
      double SPINDEXER_GEAR_RATIO,
      double SPIN_GEAR_RATIO,
      double INDEXER_GEAR_RATIO) {
    this.spindexer = spindexer;
    this.GEAR_RATIO_DICT.put(SPINDEXER, SPINDEXER_GEAR_RATIO);
    this.GEAR_RATIO_DICT.put(IdxDevice.INDEXER, INDEXER_GEAR_RATIO);
    this.GEAR_RATIO_DICT.put(IdxDevice.SPIN, SPIN_GEAR_RATIO);
  }

  private AngularVelocity getModeVelocity(IdxMode mode, IdxDevice device) {
    return switch (mode) {
      case OFF -> convertSpeed(SPINDEXER, device, RotationsPerSecond.of(0));
      case FEED ->
          convertSpeed(
              SPINDEXER, device, RotationsPerSecond.of(IdxModeParamsNT.feedSpindexer.getValue()));
      case REVERSE ->
          convertSpeed(
              SPINDEXER, device, RotationsPerSecond.of(IdxModeParamsNT.revSpindexer.getValue()));
    };
  }

  private AngularVelocity convertSpeed(IdxDevice from, IdxDevice to, AngularVelocity speed) {
    return speed.div(GEAR_RATIO_DICT.get(from)).times(GEAR_RATIO_DICT.get(to));
  }

  public void setDefaultCommand() {
    spindexer.setDefaultCommand(
        spindexer.runVelocity(() -> getModeVelocity(IdxMode.OFF, SPINDEXER)));
  }

  public Command runMode(IdxMode mode) {
    return runMode(() -> mode);
  }

  public Command runMode(Supplier<IdxMode> modeSupplier) {
    return spindexer.runVelocity(() -> getModeVelocity(modeSupplier.get(), SPINDEXER));
  }

  public Command runVelocity(AngularVelocity velocity, IdxDevice device) {
    return spindexer.runVelocity(convertSpeed(device, SPINDEXER, velocity));
  }

  public Command runOpenLoop(double DutyCycle) {
    return spindexer.runDutyCycle(DutyCycle);
  }

  protected void logState() {
    Logger.recordOutput(
        IdxConfig.SPINDEXER + "/" + IdxConfig.SPIN + "_speed",
        convertSpeed(SPINDEXER, SPIN, spindexer.getVelocity()));
    Logger.recordOutput(
        IdxConfig.SPINDEXER + "/" + IdxConfig.INDEXER + "_speed",
        convertSpeed(SPINDEXER, INDEXER, spindexer.getVelocity()));
  }

  public enum IdxMode {
    OFF,
    FEED,
    REVERSE
  }

  public enum IdxDevice {
    SPIN,
    INDEXER,
    SPINDEXER
  }
}
