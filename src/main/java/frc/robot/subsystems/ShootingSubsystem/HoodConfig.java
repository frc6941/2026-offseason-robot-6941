package frc.robot.subsystems.ShootingSubsystem;

import com.ctre.phoenix6.signals.InvertedValue;

import lib.ironpulse.subsystem.SubsystemConfig;
import lib.ntext.NTParameter;

public class HoodConfig {
    private HoodConfig() {}
    public static final String NAME = "Elevator";
    public static final String CANIVORE_CAN_BUS_NAME = "6941Canivore0";

    // Local hardware constants
    private static final int HOOD_MOTOR_MAIN_ID = 99;

    private static final int HOOD_GEAR_RATIO = 0;
    public static final SubsystemConfig HOOD_CONFIG = 
        SubsystemConfig.builder()
            .name(NAME)
            .mainBus(CANIVORE_CAN_BUS_NAME)
            .mainId(HOOD_MOTOR_MAIN_ID)
            .motorInvertedValue(InvertedValue.Clockwise_Positive)
            .SensorToMechanismRatio(HOOD_GEAR_RATIO)
            .zeroingConfig(SubsystemConfig.ZeroingConfig.builder()
                .zeroingCurrentLimit(50)
                .zeroingFilterSize(5)
                .zeroingVoltage(1)
                .build())
            .simConfig(SubsystemConfig.SimConfig.builder()
                .gearRatio(HOOD_GEAR_RATIO)
                .build())
            .build();
    
        @NTParameter(tableName = "Params/" + NAME)
        public static final class HoodParams {
    
            public static final double kP = 3.75;
            public static final double kI = 0.0;
            public static final double kD = 0.0;
            public static final double kV = 0.1308;
            public static final double kA = 0.0068;
            public static final double kS = 0.13;
    
            // Motion Magic - Up (slower, controlled)
            public static final double motionMagicVelRPS = 250.0;
            public static final double motionMagicAccelRPS2 = 600.0;
            public static final double motionMagicJerkRPS3 = 0.0;
    
            // Tolerances / behavior
            public static final double atGoalToleranceDegrees = 1;

        }

}
