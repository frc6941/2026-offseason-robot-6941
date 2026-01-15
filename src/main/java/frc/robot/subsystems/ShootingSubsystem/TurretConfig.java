package frc.robot.subsystems.ShootingSubsystem;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import lib.ironpulse.subsystem.SubsystemConfig;
import lib.ntext.NTParameter;

public class TurretConfig {
    private TurretConfig() {}

    public static final String NAME = "Turret";
    public static final String CANIVORE_CAN_BUS_NAME = "6941Canivore0";

    private static final int TURRET_MOTOR_MAIN_ID = 96;
    private static final double TURRET_GEAR_RATIO = 40.0;

    public static final SubsystemConfig TURRET_CONFIG =
            SubsystemConfig.builder()
                    .name(NAME)
                    .mainBus(CANIVORE_CAN_BUS_NAME)
                    .mainId(TURRET_MOTOR_MAIN_ID)
                    .SensorToMechanismRatio(TURRET_GEAR_RATIO)
                    .motorInvertedValue(InvertedValue.Clockwise_Positive)
                    .kSValue(StaticFeedforwardSignValue.UseVelocitySign)
                    .forwardSoftLimitDegrees(Degrees.of(0))
                    .reverseSoftLimitDegrees(Degrees.of(0))
                    .simConfig(
                            SubsystemConfig.SimConfig.builder()
                                    .gearRatio(TURRET_GEAR_RATIO)
                                    .build())
                    .build();

        @NTParameter(tableName = "Params/" + NAME)
        public static final class TurretParams {        
            public static final double kP = 3.75;
            public static final double kI = 0.0;
            public static final double kD = 0.0;
            public static final double kV = 0.1308;
            public static final double kA = 0.0068;
            public static final double kS = 0.13;       
            // Tolerances / behavior
            public static final double velocityAtGoalToleranceRPS = 1;
        }
}
