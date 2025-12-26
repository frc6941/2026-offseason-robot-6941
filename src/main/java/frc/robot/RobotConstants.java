package frc.robot;

/**
 * Robot-wide constants that are used across multiple subsystems.
 * Constants that need tuning are using NTParameter annotation.
 */
public final class RobotConstants {
    // Robot timing constants
    public static final double LOOPER_DT = 0.01; // 50Hz control loop

    // CAN bus configuration
    public static final String CANIVORE_CAN_BUS_NAME = "10541Canivore0";

    // Hardware device IDs
    public static final int PIGEON_ID = 14; // Pigeon2 IMU device ID

    private RobotConstants() {
        // Prevent instantiation
    }
}
