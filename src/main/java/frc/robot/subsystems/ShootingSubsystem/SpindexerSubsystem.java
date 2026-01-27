package frc.robot.subsystems.ShootingSubsystem;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Configs.IdxModeParamsNT;
import java.util.function.Supplier;
import lib.ironpulse.io.MotorIO;
import lib.ironpulse.io.MotorInputsAutoLogged;
import lib.ironpulse.subsystem.velocity.VelocityMotorSubsystem;

public class SpindexerSubsystem {
    public enum IdxMode {
        OFF,
        FEED,
        REVERSE
    }

    private final VelocityMotorSubsystem<MotorInputsAutoLogged, MotorIO> spin;
    private final VelocityMotorSubsystem<MotorInputsAutoLogged, MotorIO> vert;
    private final VelocityMotorSubsystem<MotorInputsAutoLogged, MotorIO> horiz;

    public SpindexerSubsystem(
            VelocityMotorSubsystem<MotorInputsAutoLogged, MotorIO> spin,
            VelocityMotorSubsystem<MotorInputsAutoLogged, MotorIO> vert,
            VelocityMotorSubsystem<MotorInputsAutoLogged, MotorIO> horiz) {
        this.spin = spin;
        this.vert = vert;
        this.horiz = horiz;
    }

    public void setDefaultCommand() {
        spin.setDefaultCommand(spin.runVelocity(() -> getSpinVelocity(IdxMode.OFF)));
        vert.setDefaultCommand(vert.runVelocity(() -> getVerticalVelocity(IdxMode.OFF)));
        horiz.setDefaultCommand(horiz.runVelocity(() -> getHorizontalVelocity(IdxMode.OFF)));
    }

    public Command runMode(IdxMode mode) {
        return runMode(() -> mode);
    }

    public Command runMode(Supplier<IdxMode> modeSupplier) {
        return Commands.parallel(
                spin.runVelocity(() -> getSpinVelocity(modeSupplier.get())),
                vert.runVelocity(() -> getVerticalVelocity(modeSupplier.get())),
                horiz.runVelocity(() -> getHorizontalVelocity(modeSupplier.get())));
    }

    public Command runVelocity(
            AngularVelocity spinVelocity,
            AngularVelocity verticalVelocity,
            AngularVelocity horizontalVelocity) {
        return Commands.parallel(
                spin.runVelocity(spinVelocity),
                vert.runVelocity(verticalVelocity),
                horiz.runVelocity(horizontalVelocity));
    }

    public Command runOpenLoop(
            double spinDutyCycle, double verticalDutyCycle, double horizontalDutyCycle) {
        return Commands.parallel(
                spin.runDutyCycle(spinDutyCycle),
                vert.runDutyCycle(verticalDutyCycle),
                horiz.runDutyCycle(horizontalDutyCycle));
    }

    private static AngularVelocity getSpinVelocity(IdxMode mode) {
        return switch (mode) {
            case OFF -> RotationsPerSecond.of(0);
            case FEED -> RotationsPerSecond.of(IdxModeParamsNT.feedSpin.getValue());
            case REVERSE -> RotationsPerSecond.of(IdxModeParamsNT.revSpin.getValue());
        };
    }

    private static AngularVelocity getVerticalVelocity(IdxMode mode) {
        return switch (mode) {
            case OFF -> RotationsPerSecond.of(0);
            case FEED -> RotationsPerSecond.of(IdxModeParamsNT.feedVert.getValue());
            case REVERSE ->
                    RotationsPerSecond.of(IdxModeParamsNT.revVert.getValue());
        };
    }

    private static AngularVelocity getHorizontalVelocity(IdxMode mode) {
        return switch (mode) {
            case OFF -> RotationsPerSecond.of(0);
            case FEED -> RotationsPerSecond.of(IdxModeParamsNT.feedHoriz.getValue());
            case REVERSE ->
                    RotationsPerSecond.of(IdxModeParamsNT.revHoriz.getValue());
        };
    }
}
