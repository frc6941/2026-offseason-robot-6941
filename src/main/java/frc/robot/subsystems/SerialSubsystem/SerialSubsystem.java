package frc.robot.subsystems.SerialSubsystem;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class SerialSubsystem extends SubsystemBase {

    private SerialPort arduino;
    private final StringBuilder rxBuffer = new StringBuilder();
    private boolean enabled = false;

    @Getter private double speed = 0.0;
    @Getter private String lastException = "";
    private double lastRxTime = 0.0;

    private static final double TIMEOUT_SEC = 0.5;

    public SerialSubsystem() {
        try {
            arduino = new SerialPort(1000000, SerialPort.Port.kUSB);
            arduino.setTimeout(0.0);
            arduino.setReadBufferSize(256);
        } catch (Exception e) {
            arduino = null;
        }
    }

    public void enable() {
        enabled = true;
        rxBuffer.setLength(0);
        lastException = "";
        lastRxTime = 0.0;
    }

    public void disable() {
        enabled = false;
        rxBuffer.setLength(0);
    }

    @Override
    public void periodic() {
        if (!enabled || arduino == null) return;

        int available = arduino.getBytesReceived();
        if (available <= 0) return;

        byte[] data = arduino.read(available);
        for (byte b : data) {
            processByte((char) b);
        }
    }

    private void processByte(char c) {
        if (c == '<') {
            rxBuffer.setLength(0);
        } else if (c == '>') {
            parseFrame(rxBuffer.toString().trim());
            rxBuffer.setLength(0);
        } else {
            rxBuffer.append(c);
        }
    }

    private void parseFrame(String frame) {
        lastRxTime = Timer.getFPGATimestamp();

        if (frame.startsWith("Speed(m/s):")) {
            try {
                speed = Double.parseDouble(frame.substring("Speed(m/s):".length()));
                lastException = "";
            } catch (Exception ignored) {
            }
            return;
        }

        lastException = frame;
    }

    public boolean isTimedOut() {
        if (lastRxTime == 0) return true;
        return Timer.getFPGATimestamp() - lastRxTime > TIMEOUT_SEC;
    }

    public void log() {
        Logger.recordOutput("Arduino/Speed", speed);
        Logger.recordOutput("Arduino/Exception", lastException);
        Logger.recordOutput("Arduino/TimedOut", isTimedOut());
        Logger.recordOutput("Arduino/Enabled", enabled);
    }
}
