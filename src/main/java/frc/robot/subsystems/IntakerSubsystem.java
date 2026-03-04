package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Configs.IntakerExtensionParamsNT;
import frc.robot.subsystems.Configs.IntakerRollerParamsNT;
import lib.ironpulse.io.MotorIO;
import lib.ironpulse.io.MotorInputsAutoLogged;
import lib.ironpulse.subsystem.position.PositionMotorSubsystem;
import lib.ironpulse.subsystem.velocity.VelocityMotorSubsystem;

public class IntakerSubsystem {
    private VelocityMotorSubsystem<MotorInputsAutoLogged, MotorIO> roller;
    private PositionMotorSubsystem<MotorInputsAutoLogged, MotorIO, Distance> extension;
    private enum IntakeMode {
        INTAKING,
        EXTENDED_IDLE,
        RETRACTED
    }

    private IntakeMode desiredMode = IntakeMode.EXTENDED_IDLE;

    public IntakerSubsystem(
            VelocityMotorSubsystem<MotorInputsAutoLogged, MotorIO> roller,
            PositionMotorSubsystem<MotorInputsAutoLogged, MotorIO, Distance> extension) {
        this.roller = roller;
        this.extension = extension;
    }

    public void setDefaultCommand() {
        roller.setDefaultCommand(roller.runStop());
    }

    public Command runIntake() {
        return Commands.parallel(
                roller.runVelVolt(
                        () -> RotationsPerSecond.of(IntakerRollerParamsNT.intakeVelRPS.getValue())),
                extension.runMotionMagic(
                        () -> Meters.of(IntakerExtensionParamsNT.deployPosMeters.getValue())),
                Commands.runOnce(() -> desiredMode = IntakeMode.INTAKING));
    }

    public Command runExtendedIdle() {
        return Commands.parallel(
                roller.runStop(),
                extension.runMotionMagic(
                        () -> Meters.of(IntakerExtensionParamsNT.deployPosMeters.getValue())),
                Commands.runOnce(() -> desiredMode = IntakeMode.EXTENDED_IDLE));
    }

    public Command runRetract() {
        return Commands.parallel(
                roller.runStop(),
                extension.runMotionMagic(
                        () -> Meters.of(IntakerExtensionParamsNT.retractPosMeters.getValue())),
                Commands.runOnce(() -> desiredMode = IntakeMode.RETRACTED));
    }

    public Command toggleIntake() {
        return Commands.either(
                runExtendedIdle(), runIntake(), () -> desiredMode == IntakeMode.INTAKING);
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
        return Commands.parallel(
                        roller.runVelVolt(
                                () ->
                                        RotationsPerSecond.of(
                                                IntakerRollerParamsNT.intakeVelRPS.getValue())),
                        extension.runPosition(
                                () -> Meters.of(IntakerExtensionParamsNT.feedPosMeters.getValue())))
                .finallyDo(
                        () -> {
                            switch (desiredMode) {
                                case INTAKING:
                                    CommandScheduler.getInstance().schedule(runIntake());
                                    break;
                                case EXTENDED_IDLE:
                                    CommandScheduler.getInstance().schedule(runExtendedIdle());
                                    break;
                                case RETRACTED:
                                    CommandScheduler.getInstance().schedule(runRetract());
                                    break;
                                default:
                                    break;
                            }
                        });
    }
}
