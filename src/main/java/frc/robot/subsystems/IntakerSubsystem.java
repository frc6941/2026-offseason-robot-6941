package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.Robot;
import frc.robot.subsystems.Configs.IntakeConfig;
import frc.robot.subsystems.Configs.IntakerExtensionParamsNT;
import frc.robot.subsystems.Configs.IntakerRollerParamsNT;
import lib.ironpulse.io.MotorIO;
import lib.ironpulse.io.MotorInputsAutoLogged;
import lib.ironpulse.subsystem.position.PositionMotorSubsystem;
import lib.ironpulse.subsystem.velocity.VelocityMotorSubsystem;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;

public class IntakerSubsystem {
    private VelocityMotorSubsystem<MotorInputsAutoLogged, MotorIO> roller;
    private PositionMotorSubsystem<MotorInputsAutoLogged, MotorIO, Distance> extension;

    private double currentFilterValue = 0.0;
    private LinearFilter currentFilter;

    public enum IntakeMode {
        INTAKING,
        EXTENDED_IDLE,
        RETRACTED,
        FEEDING,
        EXTENDED_REVERSE
    }

    @Getter
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
                Commands.either(
                                roller.runVelVolt(
                                                () ->
                                                        RotationsPerSecond.of(
                                                                currentMode
                                                                                == IntakeMode
                                                                                        .EXTENDED_REVERSE
                                                                        ? IntakerRollerParamsNT
                                                                                .outtakeVelRPS
                                                                                .getValue()
                                                                        : IntakerRollerParamsNT
                                                                                .intakeVelRPS
                                                                                .getValue()))
                                        .until(
                                                () ->
                                                        currentMode != IntakeMode.INTAKING
                                                                && currentMode != IntakeMode.FEEDING
                                                                && currentMode
                                                                        != IntakeMode
                                                                                .EXTENDED_REVERSE),
                                roller.runStop()
                                        .until(
                                                () ->
                                                        currentMode == IntakeMode.INTAKING
                                                                || currentMode == IntakeMode.FEEDING
                                                                || currentMode
                                                                        == IntakeMode
                                                                                .EXTENDED_REVERSE),
                                () ->
                                        currentMode == IntakeMode.INTAKING
                                                || currentMode == IntakeMode.FEEDING
                                                || currentMode == IntakeMode.EXTENDED_REVERSE)
                        .repeatedly());
        extension.setDefaultCommand(
                extension.runMotionMagic(
                        () -> {
                            switch (currentMode) {
                                case INTAKING:
                                    return Meters.of(
                                            IntakerExtensionParamsNT.deployPosMeters.getValue());
                                case EXTENDED_IDLE:
                                case EXTENDED_REVERSE:
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
                    if (currentMode != IntakeMode.FEEDING
                            && currentMode != IntakeMode.EXTENDED_REVERSE) {
                        currentMode = IntakeMode.INTAKING;
                    }
                });
    }

    public Command runExtendedIdle() {
        return Commands.runOnce(
                () -> {
                    fallbackMode = IntakeMode.EXTENDED_IDLE;
                    if (currentMode != IntakeMode.FEEDING
                            && currentMode != IntakeMode.EXTENDED_REVERSE) {
                        currentMode = IntakeMode.EXTENDED_IDLE;
                    }
                });
    }

    public Command runRetract() {
        return Commands.runOnce(
                () -> {
                    fallbackMode = IntakeMode.RETRACTED;
                    if (currentMode != IntakeMode.FEEDING
                            && currentMode != IntakeMode.EXTENDED_REVERSE) {
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

    public Command runExtendedReverse() {
        return Commands.startEnd(
                () -> currentMode = IntakeMode.EXTENDED_REVERSE, () -> currentMode = fallbackMode);
    }

    public Command zeroCommand() {
        return extension.zeroCommand();
    }

    public Command outZeroCommand() {
        double zeroVoltage =
                MathUtil.clamp(
                        -IntakeConfig.INTAKER_EXTENSION_CONFIG.zeroingConfig.zeroingVoltage,
                        -12.0d,
                        12.0d);

        Command realZero =
                Commands.runOnce(
                                () -> {
                                    extension.setEnableSoftLimits(false, false);
                                    currentFilter =
                                            LinearFilter.movingAverage(
                                                    IntakeConfig.INTAKER_EXTENSION_CONFIG
                                                            .zeroingConfig
                                                            .zeroingFilterSize);
                                    currentFilterValue = 0.0;
                                },
                                extension)
                        .andThen(
                                Commands.deadline(
                                        Commands.run(
                                                        () ->
                                                                currentFilterValue =
                                                                        currentFilter.calculate(
                                                                                extension
                                                                                        .getStatorCurrent()
                                                                                        .in(Amps)))
                                                .until(
                                                        () ->
                                                                Math.abs(currentFilterValue)
                                                                        > IntakeConfig
                                                                                .INTAKER_EXTENSION_CONFIG
                                                                                .zeroingConfig
                                                                                .zeroingCurrentLimit),
                                        extension.runVoltage(() -> zeroVoltage)))
                        .andThen(
                                Commands.runOnce(
                                        () ->
                                                extension.setCurrPos(
                                                        Meters.of(
                                                                IntakerExtensionParamsNT
                                                                        .deployPosMeters
                                                                        .getValue())),
                                        extension));

        Command simZero =
                Commands.runOnce(
                        () ->
                                extension.setCurrPos(
                                        Meters.of(
                                                IntakerExtensionParamsNT.deployPosMeters
                                                        .getValue())),
                        extension);

        return new ConditionalCommand(realZero, simZero, Robot::isReal);
    }
}
