package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import static frc.robot.RobotConstants.*;

import com.ctre.phoenix6.signals.InvertedValue;
import lib.ironpulse.subsystem.SubsystemConfig;
import lib.ironpulse.subsystem.SubsystemConfig.SimConfig;
import lib.ntext.NTParameter;

/** Constants and NT-backed params for {@link FlywheelSubsystem}. */
public final class FlywheelConfig {
    private FlywheelConfig() {}

    public static final String NAME = "Flywheel";

    // Local hardware constants (copied/adapted from provided snippet)
    private static final int INTAKE_PIVOT_MOTOR_ID = 22;
    // private static final int INTAKE_PIVOT_ENCODER_ID = 17;
    // private static final double INTAKE_PIVOT_ROTOR_ENCODER_RATIO = 80.1818181818;
    // // (45/11)*(56/20)*(56/8)
    // private static final double INTAKE_PIVOT_ENCODER_OFFSET = -0.382568; //
    // -0.132568 - 0.25

    // Mechanism config
    public static final SimConfig SIM_CONFIG =
            SubsystemConfig.SimConfig.builder()
                    .MOI(KilogramSquareMeters.of(0.00035))
                    .gearRatio(6.75)
                    .stdvs(new double[] {0.02, 0.02})
                    .build();

    public static final SubsystemConfig CONFIG =
            SubsystemConfig.builder()
                    .name(NAME)
                    .mainId(INTAKE_PIVOT_MOTOR_ID)
                    .mainBus(CANIVORE_CAN_BUS_NAME)
                    .motorInvertedValue(InvertedValue.Clockwise_Positive)
                    .SensorToMechanismRatio(1.0)
                    // .enableRemoteCANcoder(true)
                    // .remoteCANcoder(SubsystemConfig.RemoteCANcoder.builder()
                    // .id(INTAKE_PIVOT_ENCODER_ID)
                    // .bus(CANIVORE_CAN_BUS_NAME)
                    // .magnetOffset(INTAKE_PIVOT_ENCODER_OFFSET)
                    // .rotorToSensorRatio(INTAKE_PIVOT_ROTOR_ENCODER_RATIO)
                    // .sensorDirection(SensorDirectionValue.CounterClockwise_Positive)
                    // .feedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
                    // .useContinousWrap(false)
                    // .build())
                    .simConfig(SIM_CONFIG)
                    .build();

    @NTParameter(tableName = "Params/" + NAME)
    public static final class FlywheelParams {
        // PID/FF gains for velocity control
        public static final double kP = 0.2;
        public static final double kI = 0.0;
        public static final double kD = 0.001;
        public static final double kV = 0.003;
        public static final double kA = 0.115;
        public static final double kS = 0.285;

        // Tolerances / behavior
        public static final double velocityAtGoalToleranceRPS = 2.0;
        public static final boolean isBrake = false; // Flywheels typically use coast
    }
}
