package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.subsystems.Configs.IntakeConfig;
import frc.robot.subsystems.Configs.IntakerExtensionParamsNT;
import frc.robot.subsystems.Configs.IntakerRollerParamsNT;
import lib.ironpulse.io.MotorIO;
import lib.ironpulse.io.MotorInputsAutoLogged;
import lib.ironpulse.subsystem.position.PositionMotorSubsystem;
import lib.ironpulse.subsystem.velocity.VelocityMotorSubsystem;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;

public class IntakerSubsystem extends SubsystemBase {

    private final VelocityMotorSubsystem<MotorInputsAutoLogged, MotorIO> roller;
    private final VelocityMotorSubsystem<MotorInputsAutoLogged, MotorIO> downRoller;
    private final PositionMotorSubsystem<MotorInputsAutoLogged, MotorIO, Distance> extension;

    private double currentRollerRPS = 0.0;
    private Timer zeroTimer = new Timer();
    private double currentFilterValue = 0.0;
    private LinearFilter currentFilter;

    @AutoLogOutput(key = "IntakerRoller/autoZeroRunning")
    private boolean autoOutZeroRunning = false;

    @Getter
    @AutoLogOutput(key = "IntakerRoller/state")
    private IntakeMode currentMode = IntakeMode.RETRACTED;

    @AutoLogOutput(key = "IntakerRoller/fallbackState")
    private IntakeMode fallbackMode = IntakeMode.RETRACTED;

    public IntakerSubsystem(
            VelocityMotorSubsystem<MotorInputsAutoLogged, MotorIO> roller,
            VelocityMotorSubsystem<MotorInputsAutoLogged, MotorIO> downRoller,
            PositionMotorSubsystem<MotorInputsAutoLogged, MotorIO, Distance> extension) {

        this.roller = roller;
        this.downRoller = downRoller;
        this.extension = extension;
        RobotModeTriggers.teleop().onTrue(Commands.runOnce(() -> autoOutZeroRunning = false));
    }

    private boolean isDeployMode() {
        return (currentMode == IntakeMode.INTAKING
                || currentMode == IntakeMode.EXTENDED_IDLE
                || currentMode == IntakeMode.EXTENDED_REVERSE);
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
                                                                                .EXTENDED_REVERSE
                                                                && currentMode
                                                                        != IntakeMode
                                                                                .RETRACTED_FEEDING),
                                roller.runStop()
                                        .until(
                                                () ->
                                                        currentMode == IntakeMode.INTAKING
                                                                || currentMode == IntakeMode.FEEDING
                                                                || currentMode
                                                                        == IntakeMode
                                                                                .EXTENDED_REVERSE
                                                                || currentMode
                                                                        == IntakeMode
                                                                                .RETRACTED_FEEDING),
                                () ->
                                        currentMode == IntakeMode.INTAKING
                                                || currentMode == IntakeMode.FEEDING
                                                || currentMode == IntakeMode.EXTENDED_REVERSE
                                                || currentMode == IntakeMode.RETRACTED_FEEDING)
                        .repeatedly());

        downRoller.setDefaultCommand(
                Commands.either(
                                downRoller
                                        .runVelVolt(
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
                                                                                .EXTENDED_REVERSE
                                                                && currentMode
                                                                        != IntakeMode
                                                                                .RETRACTED_FEEDING),
                                downRoller
                                        .runStop()
                                        .until(
                                                () ->
                                                        currentMode == IntakeMode.INTAKING
                                                                || currentMode == IntakeMode.FEEDING
                                                                || currentMode
                                                                        == IntakeMode
                                                                                .EXTENDED_REVERSE
                                                                || currentMode
                                                                        == IntakeMode
                                                                                .RETRACTED_FEEDING),
                                () ->
                                        currentMode == IntakeMode.INTAKING
                                                || currentMode == IntakeMode.FEEDING
                                                || currentMode == IntakeMode.EXTENDED_REVERSE
                                                || currentMode == IntakeMode.RETRACTED_FEEDING)
                        .repeatedly());

        extension.setDefaultCommand(
                extension.runMotionMagic(
                        () ->
                                switch (currentMode) {
                                    case INTAKING -> Meters.of(
                                            IntakerExtensionParamsNT.deployPosMeters.getValue());
                                    case EXTENDED_IDLE, EXTENDED_REVERSE -> Meters.of(
                                            IntakerExtensionParamsNT.deployPosMeters.getValue());
                                    case RETRACTED -> Meters.of(
                                            IntakerExtensionParamsNT.retractPosMeters.getValue());
                                    case FEEDING -> Meters.of(
                                            IntakerExtensionParamsNT.feedPosMeters.getValue());
                                    case RETRACTED_FEEDING -> Meters.of(
                                            IntakerExtensionParamsNT.retractedFeedPosMeters
                                                    .getValue());
                                    default -> Meters.of(
                                            IntakerExtensionParamsNT.retractPosMeters.getValue());
                                }));
    }

    public Command runIntakeRPS() {
        Timer timer = new Timer();
        double startRPS =
                IntakerRollerParamsNT.intakeVelRPS.getValue()
                        * IntakerRollerParamsNT.slowIntakeMultiplier.getValue();
        double targetRPS = IntakerRollerParamsNT.intakeVelRPS.getValue();

        return new FunctionalCommand(
                () -> {
                    timer.restart();
                    runRollersRPS(startRPS);
                },
                () -> {
                    double stepSeconds =
                            Math.max(0.02, IntakerRollerParamsNT.intakeRPSStepSeconds.getValue());
                    double stepCount = Math.floor(timer.get() / stepSeconds);
                    double rpsMagnitude =
                            Math.min(
                                    Math.abs(targetRPS),
                                    Math.abs(startRPS)
                                            + stepCount
                                                    * IntakerRollerParamsNT.intakeRPSStep
                                                            .getValue());
                    double rps = Math.copySign(rpsMagnitude, targetRPS);
                    runRollersRPS(rps);
                },
                interrupted -> {
                    timer.stop();
                },
                () -> false,
                roller,
                downRoller);
    }

    public Command rampStopRPS() {
        Timer timer = new Timer();
        double[] startRPS = new double[1];

        return new FunctionalCommand(
                () -> {
                    startRPS[0] = currentRollerRPS;
                    timer.restart();
                },
                () -> {
                    runRollersRPS(getRampStopRPS(timer, startRPS[0]));
                },
                interrupted -> {
                    timer.stop();
                    runRollersRPS(0.0);
                },
                () -> Math.abs(getRampStopRPS(timer, startRPS[0])) <= 0.0,
                roller,
                downRoller);
    }

    public Command runOuttake() {
        return Commands.parallel(
                roller.runVelVolt(
                        () ->
                                RotationsPerSecond.of(
                                        IntakerRollerParamsNT.outtakeVelRPS.getValue())),
                downRoller.runVelVolt(
                        () ->
                                RotationsPerSecond.of(
                                        IntakerRollerParamsNT.outtakeVelRPS.getValue())));
    }

    public Command runSlowIntake() {
        return Commands.parallel(
                roller.runVelVolt(
                        () ->
                                RotationsPerSecond.of(
                                        IntakerRollerParamsNT.intakeVelRPS.getValue()
                                                * IntakerRollerParamsNT.slowIntakeMultiplier
                                                        .getValue())),
                downRoller.runVelVolt(
                        () ->
                                RotationsPerSecond.of(
                                        IntakerRollerParamsNT.intakeVelRPS.getValue()
                                                * IntakerRollerParamsNT.slowIntakeMultiplier
                                                        .getValue())));
    }

    public Command stop() {
        return Commands.parallel(roller.runStop(), downRoller.runStop());
    }

    public Command runIntake() {
        return Commands.sequence(
                Commands.runOnce(
                        () -> {
                            fallbackMode = IntakeMode.INTAKING;
                            if (currentMode != IntakeMode.FEEDING
                                    && currentMode != IntakeMode.EXTENDED_REVERSE) {
                                currentMode = IntakeMode.INTAKING;
                            }
                        }),
                Commands.parallel(
                        roller.runVelVolt(
                                () ->
                                        RotationsPerSecond.of(
                                                IntakerRollerParamsNT.intakeVelRPS.getValue())),
                        downRoller.runVelVolt(
                                () ->
                                        RotationsPerSecond.of(
                                                IntakerRollerParamsNT.intakeVelRPS.getValue()))));
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

    public Command toggleFeeding() {
        return Commands.either(
                runIntake(),
                runRetractedFeeding(),
                () -> fallbackMode == IntakeMode.RETRACTED_FEEDING);
    }

    public Command runFeed() {
        return Commands.startEnd(
                () -> currentMode = IntakeMode.FEEDING, () -> currentMode = fallbackMode);
    }

    public Command runRetractedFeeding() {
        return Commands.startEnd(
                () -> currentMode = IntakeMode.RETRACTED_FEEDING, () -> currentMode = fallbackMode);
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
                                                                                currentFilter
                                                                                        .calculate(
                                                                                                extension
                                                                                                        .getStatorCurrent()
                                                                                                        .in(
                                                                                                                Amps)))
                                                        .until(
                                                                () ->
                                                                        Math.abs(currentFilterValue)
                                                                                >= IntakeConfig
                                                                                        .INTAKER_EXTENSION_CONFIG
                                                                                        .zeroingConfig
                                                                                        .zeroingCurrentLimit),
                                                extension.runVoltage(() -> zeroVoltage))
                                        .finallyDo(
                                                interrupted -> {
                                                    if (!interrupted) {
                                                        extension.setCurrPos(
                                                                Meters.of(
                                                                        IntakerExtensionParamsNT
                                                                                .deployPosMeters
                                                                                .getValue()));
                                                    }
                                                })
                                        .onlyWhile(this::isDeployMode));

        Command simZero =
                Commands.sequence(
                        Commands.runOnce(
                                () ->
                                        extension.setCurrPos(
                                                Meters.of(
                                                        IntakerExtensionParamsNT.deployPosMeters
                                                                .getValue()))),
                        new WaitCommand(0.2),
                        Commands.runOnce(
                                () ->
                                        extension.setCurrPos(
                                                Meters.of(
                                                        IntakerExtensionParamsNT.deployPosMeters
                                                                .getValue()))));

        return new ConditionalCommand(realZero, simZero, RobotBase::isReal)
                .finallyDo(
                        () -> {
                            extension.setEnableSoftLimits(true, true);
                            autoOutZeroRunning = false;
                            zeroTimer.restart();
                        });
    }

    @Override
    public void periodic() {
        if (!isDeployMode()) {
            zeroTimer.stop();
            zeroTimer.reset();
        } else if (!zeroTimer.isRunning()) {
            zeroTimer.start();
        }
        if (!autoOutZeroRunning && zeroTimer.hasElapsed(0.3)) {
            autoOutZeroRunning = true;
            edu.wpi.first.wpilibj2.command.CommandScheduler.getInstance()
                    .schedule(
                            outZeroCommand()
                                    .withInterruptBehavior(
                                            Command.InterruptionBehavior.kCancelSelf));
            zeroTimer.restart();
        }
    }

    private void runRollersRPS(double rps) {
        currentRollerRPS = rps;
        roller.setVelVoltSetpoint(RotationsPerSecond.of(rps));
        downRoller.setVelVoltSetpoint(RotationsPerSecond.of(rps));
    }

    private double getRampStopRPS(Timer timer, double startRPS) {
        double stepRPS = Math.max(0.1, IntakerRollerParamsNT.intakeStopRPSStep.getValue());
        double stepSeconds =
                Math.max(0.02, IntakerRollerParamsNT.intakeStopRPSStepSeconds.getValue());
        double stepCount = Math.floor(timer.get() / stepSeconds);
        double rpsMagnitude = Math.max(0.0, Math.abs(startRPS) - stepCount * stepRPS);
        return Math.copySign(rpsMagnitude, startRPS);
    }

    public enum IntakeMode {
        INTAKING,
        EXTENDED_IDLE,
        RETRACTED,
        FEEDING,
        EXTENDED_REVERSE,
        RETRACTED_FEEDING
    }
}
