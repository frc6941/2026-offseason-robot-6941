package frc.robot;

/**
 * Robot-wide constants that are used across multiple subsystems.
 * Constants that need tuning are using NTParameter annotation.
 */
public final class RobotConstants {
    // Robot timing constants
    public static final double LOOPER_DT = 0.02; // 50Hz control loop
    public static final boolean disableHAL = false;
    
    // CAN bus configuration
    public static final String CANIVORE_CAN_BUS_NAME = "6941Canivore0";
    
    // Hardware device IDs
    public static final int PIGEON_ID = 14; // Pigeon2 IMU device ID
    
    private RobotConstants() {
        // Prevent instantiation
    }
}
