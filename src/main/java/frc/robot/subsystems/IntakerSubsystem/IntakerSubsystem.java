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
                roller.runVelocity(RotationsPerSecond.of(idleVelRPS)),
                extension.runPosition(Meters.of(retractPosMeters)));
    }

    public Command outtake() {
        if (extension.getCurrSetpoint() == Meters.of(retractPosMeters)) {
            return Commands.none();
        }
        return roller.runVelocity(RotationsPerSecond.of(outtakeVelRPS));
    }

    public Command intake() {
        if (extension.getCurrSetpoint() == Meters.of(retractPosMeters)) {
            return Commands.none();
        }
        return roller.runVelocity(RotationsPerSecond.of(intakeVelRPS));
    }

    public Command deployIntake() {
        return Commands.parallel(
                roller.runVelocity(RotationsPerSecond.of(intakeVelRPS)),
                extension.runPosition(Meters.of(deployPosMeters)));
    }
}
