package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Configs.IntakerExtensionParamsNT;
import frc.robot.subsystems.Configs.IntakerRollerParamsNT;
import lib.ironpulse.io.MotorIO;
import lib.ironpulse.io.MotorInputsAutoLogged;
import lib.ironpulse.subsystem.position.PositionMotorSubsystem;
import lib.ironpulse.subsystem.velocity.VelocityMotorSubsystem;
import org.littletonrobotics.junction.AutoLogOutput;

public class IntakerSubsystem {
    private VelocityMotorSubsystem<MotorInputsAutoLogged, MotorIO> roller;
    private PositionMotorSubsystem<MotorInputsAutoLogged, MotorIO, Distance> extension;

    private enum IntakeMode {
        INTAKING,
        EXTENDED_IDLE,
        RETRACTED,
        FEEDING
    }

    @AutoLogOutput(key = "IntakerRoller/state")
    private IntakeMode currentMode = IntakeMode.RETRACTED;

    @AutoLogOutput(key = "IntakerRoller/fallbackState")
    private IntakeMode fallbackMode = IntakeMode.RETRACTED;

    public IntakerSubsystem(
            VelocityMotorSubsystem<MotorInputsAutoLogged, MotorIO> roller,
            PositionMotorSubsystem<MotorInputsAutoLogged, MotorIO, Distance> extension) {
        this.roller = roller;
        this.extension = extension;
    }

    public void setDefaultCommand() {
        roller.setDefaultCommand(
                roller.runVelVolt(
                        () ->
                                RotationsPerSecond.of(
                                        currentMode == IntakeMode.INTAKING
                                                        || currentMode == IntakeMode.FEEDING
                                                ? IntakerRollerParamsNT.intakeVelRPS.getValue()
                                                : IntakerRollerParamsNT.idleVelRPS.getValue())));
        extension.setDefaultCommand(
                extension.runMotionMagic(
                        () -> {
                            switch (currentMode) {
                                case INTAKING:
                                    return Meters.of(
                                            IntakerExtensionParamsNT.deployPosMeters.getValue());
                                case EXTENDED_IDLE:
                                    return Meters.of(
                                            IntakerExtensionParamsNT.deployPosMeters.getValue());
                                case RETRACTED:
                                    return Meters.of(
                                            IntakerExtensionParamsNT.retractPosMeters.getValue());
                                case FEEDING:
                                    return Meters.of(
                                            IntakerExtensionParamsNT.feedPosMeters.getValue());
                                default:
                                    return Meters.of(
                                            IntakerExtensionParamsNT.retractPosMeters.getValue());
                            }
                        }));
    }

    public Command runIntake() {
        return Commands.runOnce(
                () -> {
                    fallbackMode = IntakeMode.INTAKING;
                    if (currentMode != IntakeMode.FEEDING) {
                        currentMode = IntakeMode.INTAKING;
                    }
                });
    }

    public Command runExtendedIdle() {
        return Commands.runOnce(
                () -> {
                    fallbackMode = IntakeMode.EXTENDED_IDLE;
                    if (currentMode != IntakeMode.FEEDING) {
                        currentMode = IntakeMode.EXTENDED_IDLE;
                    }
                });
    }

    public Command runRetract() {
        return Commands.runOnce(
                () -> {
                    fallbackMode = IntakeMode.RETRACTED;
                    if (currentMode != IntakeMode.FEEDING) {
                        currentMode = IntakeMode.RETRACTED;
                    }
                });
    }

    public Command toggleIntake() {
        return Commands.either(
                runExtendedIdle(), runIntake(), () -> fallbackMode == IntakeMode.INTAKING);
    }

    //     public Command runFeed() {
    //         return Commands.runOnce(
    //                         () -> feedOscillationStartTime = Timer.getFPGATimestamp(), extension)
    //                 .andThen(
    //                         Commands.parallel(
    //                                 roller.runVelVolt(RotationsPerSecond.of(intakeVelRPS)),
    //                                 extension.runPosition(
    //                                         () -> {
    //                                             double t =
    //                                                     Timer.getFPGATimestamp()
    //                                                             - feedOscillationStartTime;
    //                                             double rate =
    //
    // IntakerExtensionParamsNT.feedOscillationRateHz
    //                                                             .getValue();
    //                                             double deploy =
    //                                                     IntakerExtensionParamsNT.deployPosMeters
    //                                                             .getValue();
    //                                             double feed =
    //                                                     IntakerExtensionParamsNT.feedPosMeters
    //                                                             .getValue();
    //                                             double pos =
    //                                                     feed
    //                                                             + (deploy - feed)
    //                                                                     * (1
    //                                                                             + Math.sin(
    //                                                                                     2 *
    // Math.PI
    //                                                                                             *
    // rate
    //                                                                                             *
    // t))
    //                                                                     / 2;
    //                                             return Meters.of(pos);
    //                                         })));
    //     }
    public Command runFeed() {
        return Commands.startEnd(
                () -> currentMode = IntakeMode.FEEDING, () -> currentMode = fallbackMode);
    }
}
