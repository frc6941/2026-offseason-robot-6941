package frc.robot.subsystems;

import static frc.robot.RobotConstants.CANIVORE_CAN_BUS_NAME;

import com.ctre.phoenix6.signals.InvertedValue;
import lib.ironpulse.subsystem.SubsystemConfig;

public final class RollerConfigs {
    public static final SubsystemConfig intakeRollerCfg =
            SubsystemConfig.simpleMotorCfg(
                    "IntakeRoller",
                    15,
                    CANIVORE_CAN_BUS_NAME,
                    InvertedValue.CounterClockwise_Positive);
    public static final SubsystemConfig indexRollerCfg =
            SubsystemConfig.simpleMotorCfg(
                    "IndexRoller", 19, CANIVORE_CAN_BUS_NAME, InvertedValue.Clockwise_Positive);
    public static final SubsystemConfig EERollerCfg =
            SubsystemConfig.simpleMotorCfg(
                    "EERoller", 22, CANIVORE_CAN_BUS_NAME, InvertedValue.Clockwise_Positive);
}
