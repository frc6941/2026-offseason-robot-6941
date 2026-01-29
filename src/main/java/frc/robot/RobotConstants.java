package frc.robot;

import com.ctre.phoenix6.CANBus;

/**
 * Robot-wide constants that are used across multiple subsystems. Constants that need tuning are
 * using NTParameter annotation.
 */
public final class RobotConstants {
    // Robot timing constants
    public static final double LOOPER_DT = 0.01; // 50Hz control loop

    // CAN bus configuration
    public static final String CANIVORE_CAN_BUS_NAME = "6941Canivore0";

    public static final CANBus canivoreBus = new CANBus(CANIVORE_CAN_BUS_NAME);

    // Hardware device IDs
    public static final int PIGEON_ID = 14; // Pigeon2 IMU device ID

    public static final boolean disableHAL = true;

    private RobotConstants() {
        // Prevent instantiation
    }
}
