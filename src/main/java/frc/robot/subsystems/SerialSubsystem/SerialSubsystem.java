package frc.robot.subsystems.SerialSubsystem;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;

public class SerialSubsystem extends SubsystemBase {
    private final I2C i2c;
    private static final int ARDUINO_ADDR = 0x08;

    @Getter
    private double speed = 0.0;
    private double previousSpeed = 0.0;  // Added to track previous speed
    private boolean enabled = false;
    private double lastRxTime = 0.0;

    public SerialSubsystem() {
        // 使用 RoboRIO 上的 Onboard I2C 接口
        i2c = new I2C(I2C.Port.kOnboard, ARDUINO_ADDR);
    }

    public boolean isSpeedChanged() {
        return Math.abs(speed - previousSpeed) > 1e-6;  // Using small epsilon for floating point comparison
    }

    public void enable() {
        enabled = true;
    }

    public void disable() {
        enabled = false;
    }

    @Override
    public void periodic() {
        if (!enabled) return;

        byte[] data = new byte[4];
        if (!i2c.readOnly(data, 4)) {
            float receivedSpeed = ByteBuffer.wrap(data)
                                            .order(ByteOrder.LITTLE_ENDIAN)
                                            .getFloat();
            
            if (!Float.isNaN(receivedSpeed) && !Float.isInfinite(receivedSpeed)) {
                previousSpeed = speed;  // Store current speed as previous before updating
                speed = receivedSpeed;
                lastRxTime = Timer.getFPGATimestamp();
            }
        }
    }

    public boolean isTimedOut() {
        return (Timer.getFPGATimestamp() - lastRxTime) > 0.5;
    }

    public void log() {
        Logger.recordOutput("Arduino/Speed", speed);
        Logger.recordOutput("Arduino/TimedOut", isTimedOut());
        Logger.recordOutput("Arduino/Enabled", enabled);
        Logger.recordOutput("Arduino/SpeedChanged", isSpeedChanged());  // Optional: Log this as well
    }
}