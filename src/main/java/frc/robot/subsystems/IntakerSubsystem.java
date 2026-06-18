package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RotationsPerSecond;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Configs.IntakeParamsNT;
import lib.ironpulse.io.MotorIO;
import lib.ironpulse.io.MotorInputsAutoLogged;
import lib.ironpulse.subsystem.velocity.VelocityMotorSubsystem;

public class IntakerSubsystem extends SubsystemBase {

    private final VelocityMotorSubsystem<MotorInputsAutoLogged, MotorIO> roller;
    private final VelocityMotorSubsystem<MotorInputsAutoLogged, MotorIO> topRoller;

    public IntakerSubsystem(
            VelocityMotorSubsystem<MotorInputsAutoLogged, MotorIO> roller,
            VelocityMotorSubsystem<MotorInputsAutoLogged, MotorIO> topRoller) {

        this.roller = roller;
        this.topRoller = topRoller;

        roller.setDefaultCommand(roller.runStop().repeatedly());
        topRoller.setDefaultCommand(topRoller.runStop().repeatedly());
    }

    public Command runIntake() {
        return Commands.parallel(
                roller.runVelTC(
                        () -> RotationsPerSecond.of(
                                IntakeParamsNT.intakeVelRPS)),

                topRoller.runVelTC(
                        () -> RotationsPerSecond.of(
                                IntakeParamsNT.intakeVelRPS))
        );
    }

    public Command runOuttake() {
        return Commands.parallel(
                roller.runVelTC(
                        () -> RotationsPerSecond.of(
                                IntakeParamsNT.outtakeVelRPS)),

                topRoller.runVelTC(
                        () -> RotationsPerSecond.of(
                                IntakeParamsNT.outtakeVelRPS))
        );
    }

    public Command runSlowIntake() {
        return Commands.parallel(
                roller.runVelTC(
                        () -> RotationsPerSecond.of(
                                IntakeParamsNT.intakeVelRPS * IntakeParamsNT.slowIntakeMultiplier)),

                topRoller.runVelTC(
                        () -> RotationsPerSecond.of(
                                IntakeParamsNT.intakeVelRPS * IntakeParamsNT.slowIntakeMultiplier))
        );
    }

    public Command stop() {
        return Commands.parallel(
                roller.runStop(),
                topRoller.runStop()
        );
    }
}
