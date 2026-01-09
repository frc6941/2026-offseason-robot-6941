// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.*;
import lib.ironpulse.io.BeamBreakIO;
import lib.ironpulse.io.BeamBreakIOAnalog;
import lib.ironpulse.io.BeamBreakIOSim;
import lib.ironpulse.io.MotorIO;
import lib.ironpulse.io.MotorIOSim;
import lib.ironpulse.io.MotorIOTalonFX;
import lib.ironpulse.io.MotorInputsAutoLogged;
import lib.ironpulse.subsystem.BeamBreak;
import lib.ironpulse.subsystem.MotorSubsystem;
import lib.ironpulse.subsystem.position.PositionMotorSubsystem;
import lib.ironpulse.subsystem.velocity.VelocityMotorSubsystem;
import org.littletonrobotics.junction.Logger;

public class RobotContainer {
    private final CommandXboxController driver = new CommandXboxController(0);

    private final ElevatorSubsystem elevator;
    private final VelocityMotorSubsystem<MotorInputsAutoLogged, MotorIO> flywheel;
    private final MotorSubsystem<MotorInputsAutoLogged, MotorIO> intakeRoller;
    private final MotorSubsystem<MotorInputsAutoLogged, MotorIO> indexRoller;
    private final MotorSubsystem<MotorInputsAutoLogged, MotorIO> EERoller;
    private final PositionMotorSubsystem<MotorInputsAutoLogged, MotorIO, Angle> intakePivot;
    private final BeamBreak beamBreak;

    public RobotContainer() {
        MotorIO elevatorIO;
        MotorIO flywheelIO;
        MotorIO intakePivotIO;
        BeamBreakIO beamBreakIO;
        MotorIO intakeRollerIO;
        MotorIO indexRollerIO;
        MotorIO EERollerIO;

        if (Logger.hasReplaySource()) {
            elevatorIO = new MotorIO() {};
            flywheelIO = new MotorIO() {};
            intakePivotIO = new MotorIO() {};
            beamBreakIO = new BeamBreakIO() {};
            intakeRollerIO = new MotorIO() {};
            indexRollerIO = new MotorIO() {};
            EERollerIO = new MotorIO() {};
        } else if (RobotBase.isReal()) {
            elevatorIO = new MotorIOTalonFX(ElevatorConfig.CONFIG);
            flywheelIO = new MotorIOTalonFX(FlywheelConfig.CONFIG);
            intakePivotIO = new MotorIOTalonFX(IntakePivotConfig.CONFIG);
            beamBreakIO = new BeamBreakIOAnalog(1);
            intakeRollerIO = new MotorIOTalonFX(RollerConfigs.intakeRollerCfg);
            indexRollerIO = new MotorIOTalonFX(RollerConfigs.indexRollerCfg);
            EERollerIO = new MotorIOTalonFX(RollerConfigs.EERollerCfg);
        } else {
            elevatorIO = new MotorIOSim(ElevatorConfig.CONFIG);
            flywheelIO = new MotorIOSim(FlywheelConfig.CONFIG);
            intakePivotIO = new MotorIOSim(IntakePivotConfig.CONFIG);
            beamBreakIO = new BeamBreakIOSim();
            intakeRollerIO = new MotorIOSim(RollerConfigs.intakeRollerCfg);
            indexRollerIO = new MotorIOSim(RollerConfigs.indexRollerCfg);
            EERollerIO = new MotorIOSim(RollerConfigs.EERollerCfg);
        }

        beamBreak = new BeamBreak(beamBreakIO, Seconds.of(0.01));

        elevator =
                new ElevatorSubsystem(
                        ElevatorConfig.CONFIG,
                        new MotorInputsAutoLogged(),
                        elevatorIO,
                        ElevatorParamsNT.asPositionParamSources(),
                        Meters.of(0),
                        Meters.of(ElevatorConfig.METERS_PER_ROTATION));
        flywheel =
                new VelocityMotorSubsystem<>(
                        FlywheelConfig.CONFIG,
                        new MotorInputsAutoLogged(),
                        flywheelIO,
                        FlywheelParamsNT.asVelocityParamSources());
        intakePivot =
                new PositionMotorSubsystem<>(
                        IntakePivotConfig.CONFIG,
                        new MotorInputsAutoLogged(),
                        intakePivotIO,
                        IntakePivotParamsNT.asPositionParamSources(),
                        Degrees.of(0),
                        Degrees.of(360));

        intakeRoller =
                new MotorSubsystem<>(
                        RollerConfigs.intakeRollerCfg, new MotorInputsAutoLogged(), intakeRollerIO);
        indexRoller =
                new MotorSubsystem<>(
                        RollerConfigs.indexRollerCfg, new MotorInputsAutoLogged(), indexRollerIO);
        EERoller =
                new MotorSubsystem<>(
                        RollerConfigs.EERollerCfg, new MotorInputsAutoLogged(), EERollerIO);

        configureBindings();
    }

    public void robotPeriodic() {}

    private void configureBindings() {
        driver.a().onTrue(intakePivot.runPosition(Degrees.of(180)));
        driver.b().onTrue(intakePivot.runPosition(Degrees.of(90)));
        driver.x().onTrue(intakePivot.runPosition(Degrees.of(0)));
        driver.y()
                .whileTrue(
                        Commands.parallel(
                                        intakeRoller.runDutyCycle(1.0),
                                        indexRoller.runDutyCycle(1.0),
                                        EERoller.runDutyCycle(1.0))
                                .deadlineFor(beamBreak.waitForDebounced(true))
                                .andThen(
                                        Commands.parallel(
                                                intakeRoller.runStop(),
                                                indexRoller.runStop(),
                                                EERoller.runStop())));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
