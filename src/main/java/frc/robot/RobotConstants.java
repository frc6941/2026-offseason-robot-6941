package frc.robot;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.wpilibj.RobotController;
import lib.ironpulse.utils.Logging;

/**
 * Robot-wide constants that are used across multiple subsystems. Constants that need tuning are
 * using NTParameter annotation.
 */
public final class RobotConstants {
    public static final String RIOSerial10541 = "03415993";
    // Robot timing constants
    public static final double LOOPER_DT = 0.01; // 50Hz control loop
    public static final String ROBORIO_CAN_BUS_NAME = "rio";
    public static final CANBus ROBORIO_CAN_BUS = new CANBus(ROBORIO_CAN_BUS_NAME);
    // Hardware device IDs
    public static final int PIGEON_ID = 14; // Pigeon2 IMU device ID
    public static final int LED_PORT = 0;
    public static final int LED_LENGTH = 30;
    public static boolean is10541 =
            RobotController.getSerialNumber().matches(RobotConstants.RIOSerial10541);
    // CAN bus configuration
    public static final String CANIVORE_CAN_BUS_NAME = is10541 ? "10541Canivore0" : "6941Canivore0";
    public static final CANBus CANIVORE_CAN_BUS = new CANBus(CANIVORE_CAN_BUS_NAME);
    public static boolean disableHAL = false;

    // auto robot config
    public static RobotConfig AUTO_ROBOT_CONFIG;

    static {
        try {
            AUTO_ROBOT_CONFIG = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            Logging.error("Constants", "Failed to load AUTO_ROBOT_CONFIG. %s", e.getMessage());
        }
    }

    private RobotConstants() {
        // Prevent instantiation
    }
}
