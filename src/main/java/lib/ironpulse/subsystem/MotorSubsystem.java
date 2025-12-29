package lib.ironpulse.subsystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lib.ironpulse.io.MotorIO;
import lib.ironpulse.io.MotorInputsAutoLogged;

import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

/**
 * Skeleton base for single-motor mechanisms following the AdvantageKit IO
 * style.
 *
 * - Owns IO and AutoLogged inputs
 * - Reads inputs and logs in periodic()
 * - Provides minimal open-loop helper command
 */
public class MotorSubsystem<T extends MotorInputsAutoLogged, U extends MotorIO> extends SubsystemBase {
  protected final U io;
  protected final T inputs;
  protected final SubsystemConfig config;

  public MotorSubsystem(SubsystemConfig config, T inputs, U io) {
    super(config.name);
    this.config = config;
    this.io = io;
    this.inputs = inputs;
  }

  @Override
  public void periodic() {
    io.readInputs(inputs);
    Logger.processInputs("Subsystem/" + getName(), inputs);
  }

  /**
   * Open-loop duty cycle command,([-1.0, 1.0] where 1.0 = 100%).
   */
  public Command runDutyCycle(DoubleSupplier dutyCycle) {
    return run(
        () -> io.setOpenLoopDutyCycle(dutyCycle.getAsDouble()));
  }

  /**
   * Open-loop duty cycle command, -12-12V.
   */
  public Command runVoltage(DoubleSupplier voltage) {
    return run(
        () -> io.setVoltage(MathUtil.clamp(voltage.getAsDouble(), -12.0, 12.0)));
  }

  /**
   * Stop command
   */
  public Command runStop() {
    return run(() -> io.setOpenLoopDutyCycle(0.0));
  }

  /**
   * Set neutral mode command.
   */
  public Command setNeutralMode(boolean wantsBreak) {
    return run(() -> io.setNeutralMode(wantsBreak));
  }

  public Current getStatorCurrent() {
    return Amps.of(inputs.currentStatorAmps);
  }

  public Current getSupplyCurrent() {
    return Amps.of(inputs.currentSupplyAmps);
  }

  public Voltage getMotorVoltage() {
    return Volts.of(inputs.motorVolts);
  }

  public AngularVelocity getVelocityUnitsPerSecond() {
    return Rotations.of(inputs.velocityRotPerSecond).div(Seconds.of(1));
  }
}
