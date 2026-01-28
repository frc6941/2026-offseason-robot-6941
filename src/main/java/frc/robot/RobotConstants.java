package frc.robot;

import lib.ironpulse.limelight.LimelightIOConfig;
import lib.ironpulse.limelight.LimelightSubsystemConfig;

/**
 * Robot-wide constants that are used across multiple subsystems. Constants that need tuning are
 * using NTParameter annotation.
 */
public final class RobotConstants {
    // Robot timing constants
    public static final double LOOPER_DT = 0.01; // 50Hz control loop

    // CAN bus configuration
    public static final String CANIVORE_CAN_BUS_NAME = "6941Canivore0";

    // Hardware device IDs
    public static final int PIGEON_ID = 14; // Pigeon2 IMU device ID

    public static final boolean disableHAL = true;

    private RobotConstants() {
        // Prevent instantiation
    }

    public static class LimelightConstants {
        public static final LimelightSubsystemConfig limelightSubsystemConfig =
                LimelightSubsystemConfig.builder().build();
        public static final LimelightIOConfig limelight1Config =
                LimelightIOConfig.builder()
                        .name("limelight")
                        .isLimelight4(true)
                        .mountPosition(LimelightIOConfig.MountPosition.ON_ROBOT)
                        .build();
    }
}
