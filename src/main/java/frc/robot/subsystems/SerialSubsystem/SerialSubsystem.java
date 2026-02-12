package frc.robot.subsystems.SerialSubsystem;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lib.ntext.NTParameter;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class SerialSubsystem extends SubsystemBase {
  private static final int I2C_ADDRESS = 0x08;
  private static final double TIMEOUT_SEC = 0.5;
  private static final String NAME = "Arduino";

  private final I2C i2c;
  private final byte[] rx = new byte[4];

  @Getter private Time time = Seconds.of(0.0);
  @Getter private LinearVelocity speed = MetersPerSecond.of(0.0);
  @Getter private String lastException = "";
  private double lastRxTime = 0.0;

  public SerialSubsystem() {
    i2c = new I2C(I2C.Port.kMXP, I2C_ADDRESS);
  }

  @Override
  public void periodic() {

    boolean aborted = i2c.readOnly(rx, 4);
    if (aborted) {
      lastException = "I2C read aborted";
      log();
      return;
    }

    // little-endian unsigned 32-bit microseconds
    int raw =
        (rx[0] & 0xFF) | ((rx[1] & 0xFF) << 8) | ((rx[2] & 0xFF) << 16) | ((rx[3] & 0xFF) << 24);

    long us = Integer.toUnsignedLong(raw);
    if (us == 0) {
      lastException = "Time is 0us";
      log();
      return;
    }

    time = Microseconds.of(us);
    speed = Meters.of(SerialSubsystemParamsNT.distanceMeters.getValue()).div(time);
    lastException = "";
    lastRxTime = Timer.getFPGATimestamp();
    log();
  }

  public boolean isTimedOut() {
    return lastRxTime == 0.0 || (Timer.getFPGATimestamp() - lastRxTime) > TIMEOUT_SEC;
  }

  public void log() {
    Logger.recordOutput(NAME + "/TimeSeconds", time.in(Seconds));
    Logger.recordOutput(NAME + "/SpeedMPS", speed.in(MetersPerSecond));
    Logger.recordOutput(NAME + "/Exception", lastException);
    Logger.recordOutput(NAME + "/TimedOut", isTimedOut());
  }

  @NTParameter(tableName = "Params/" + NAME)
  public static final class SerialSubsystemParams {
    public static final double distanceMeters = 0.18;
  }
}
