package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Configs.IntakeConfig.IntakerRollerParams;
import lib.ironpulse.io.MotorIO;
import lib.ironpulse.io.MotorInputsAutoLogged;
import lib.ironpulse.subsystem.velocity.VelocityMotorSubsystem;

public class IntakerSubsystem extends SubsystemBase {

    private final VelocityMotorSubsystem<MotorInputsAutoLogged, MotorIO> roller;
    private final VelocityMotorSubsystem<MotorInputsAutoLogged, MotorIO> downRoller;
    private double currentRollerRPS = 0.0;



    public IntakerSubsystem(
            VelocityMotorSubsystem<MotorInputsAutoLogged, MotorIO> roller,
            VelocityMotorSubsystem<MotorInputsAutoLogged, MotorIO> downRoller) {

        this.roller = roller;
        this.downRoller = downRoller;

        roller.setDefaultCommand(roller.runStop().repeatedly());
        downRoller.setDefaultCommand(downRoller.runStop().repeatedly());
    }


    public Command runIntakeRPS() {
        Timer timer = new Timer();

        return new FunctionalCommand(
                () -> {
                    timer.restart();
                    runRollersRPS(IntakerRollerParams.intakeStartRPS);
                },
                () -> {
                    double stepSeconds = Math.max(0.02, IntakerRollerParams.intakeRPSStepSeconds);
                    double stepCount = Math.floor(timer.get() / stepSeconds);
                    double rpsMagnitude =
                            Math.min(
                                    Math.abs(IntakerRollerParams.intakeMaxRPS),
                                    Math.abs(IntakerRollerParams.intakeStartRPS)
                                            + stepCount * IntakerRollerParams.intakeRPSStep);
                    double rps = Math.copySign(rpsMagnitude, IntakerRollerParams.intakeStartRPS);
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
                roller.runVelVolt(() -> RotationsPerSecond.of(IntakerRollerParams.outtakeVelRPS)),
                downRoller.runVelVolt(
                        () -> RotationsPerSecond.of(IntakerRollerParams.outtakeVelRPS)));
    }

    public Command runSlowIntake()   {
        return Commands.parallel(
                roller.runVelVolt(
                        () ->
                                RotationsPerSecond.of(
                                        IntakerRollerParams.intakeVelRPS
                                                * IntakerRollerParams.slowIntakeMultiplier)),
                downRoller.runVelVolt(
                        () ->
                                RotationsPerSecond.of(
                                        IntakerRollerParams.intakeVelRPS
                                                * IntakerRollerParams.slowIntakeMultiplier)));
    }

    public Command stop() {
        return Commands.parallel(roller.runStop(), downRoller.runStop());
    }

    private void runRollersRPS(double rps) {
        currentRollerRPS = rps;
        roller.setVelVoltSetpoint(RotationsPerSecond.of(rps));
        downRoller.setVelVoltSetpoint(RotationsPerSecond.of(rps));
    }

    private double getRampStopRPS(Timer timer, double startRPS) {
        double stepRPS = Math.max(0.1, IntakerRollerParams.intakeStopRPSStep);
        double stepSeconds = Math.max(0.02, IntakerRollerParams.intakeStopRPSStepSeconds);
        double stepCount = Math.floor(timer.get() / stepSeconds);
        double rpsMagnitude = Math.max(0.0, Math.abs(startRPS) - stepCount * stepRPS);
        return Math.copySign(rpsMagnitude, startRPS);
    }
}
