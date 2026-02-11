package frc.robot.subsystems.SerialSubsystem;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class SerialSubsystem extends SubsystemBase {
  private static final int I2C_ADDRESS = 0x08;
  private static final double TIMEOUT_SEC = 0.5;

  private final I2C i2c;

  @Getter private double speed = 0.0;
  @Getter private String lastException = "";
  private double lastRxTime = 0.0;

  public SerialSubsystem() {
    I2C tmp;
    try {
      tmp = new I2C(I2C.Port.kMXP, I2C_ADDRESS);
    } catch (Exception e) {
      tmp = null;
      lastException = e.getMessage();
    }
    i2c = tmp;
  }

  @Override
  public void periodic() {
    lastRxTime = Timer.getFPGATimestamp();

    log();
    if (i2c == null) return;

    byte[] rx = new byte[4];
    boolean aborted = i2c.readOnly(rx, 4);
    if (aborted) {
      lastException = "I2C read aborted";
      return;
    }

    float v;
    try {
      v = ByteBuffer.wrap(rx).order(ByteOrder.LITTLE_ENDIAN).getFloat();
    } catch (Exception e) {
      lastException = "Bad float";
      return;
    }

    speed = v;
    lastException = "";
    lastRxTime = Timer.getFPGATimestamp();

    log();
  }

  public boolean isTimedOut() {
    if (lastRxTime == 0) return true;
    return Timer.getFPGATimestamp() - lastRxTime > TIMEOUT_SEC;
  }

  public void log() {
    Logger.recordOutput("Arduino/Speed", speed);
    Logger.recordOutput("Arduino/Exception", lastException);
    Logger.recordOutput("Arduino/TimedOut", isTimedOut());
  }
}
