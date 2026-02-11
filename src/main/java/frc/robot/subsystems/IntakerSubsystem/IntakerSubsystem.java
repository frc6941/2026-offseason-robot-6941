package frc.robot.subsystems.IntakerSubsystem;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.subsystems.Configs.IntakerExtensionConfig.IntakerExtensionParams.*;
import static frc.robot.subsystems.Configs.IntakerRollerConfig.IntakerRollerParams.*;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import lib.ironpulse.io.MotorIO;
import lib.ironpulse.io.MotorInputsAutoLogged;
import lib.ironpulse.subsystem.position.PositionMotorSubsystem;
import lib.ironpulse.subsystem.velocity.VelocityMotorSubsystem;

public class IntakerSubsystem {
    private VelocityMotorSubsystem<MotorInputsAutoLogged, MotorIO> roller;
    private PositionMotorSubsystem<MotorInputsAutoLogged, MotorIO, Distance> extension;

    public IntakerSubsystem(
            VelocityMotorSubsystem<MotorInputsAutoLogged, MotorIO> roller,
            PositionMotorSubsystem<MotorInputsAutoLogged, MotorIO, Distance> extension) {
        this.roller = roller;
        this.extension = extension;
    }

    public void setDefaultCommand() {
        extension.runPosition(Meters.of(retractPosMeters));
        roller.runVelocity(RotationsPerSecond.of(idleVelRPS));
    }

    public Command retractIntake() {
        return Commands.parallel(
                        roller.runVelocity(RotationsPerSecond.of(idleVelRPS))
                                .withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf),
                        extension
                                .runPosition(Meters.of(retractPosMeters))
                                .withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf))
                .withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf);
    }

    public Command outtake() {
        return isDeployed()
                ? roller.runVelocity(RotationsPerSecond.of(outtakeVelRPS))
                        .withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf)
                : Commands.none();
    }

    public Command intake() {
        return isDeployed()
                ? roller.runVelocity(RotationsPerSecond.of(intakeVelRPS))
                        .withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf)
                : Commands.none();
    }

    public Command deployIntake() {
        return Commands.parallel(
                        roller.runVelocity(RotationsPerSecond.of(intakeVelRPS))
                                .withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf),
                        extension
                                .runPosition(Meters.of(deployPosMeters))
                                .withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf))
                .withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf);
    }

    public boolean isDeployed() {
        return extension.getCurrSetpoint().equals(Meters.of(deployPosMeters));
    }
}
