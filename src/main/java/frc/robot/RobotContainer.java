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
import lib.ironpulse.subsystem.position.PositionMotorSubsystem;
import lib.ironpulse.subsystem.velocity.VelocityMotorSubsystem;
import org.littletonrobotics.junction.Logger;

public class RobotContainer {
    private final CommandXboxController driver = new CommandXboxController(0);

    private final ElevatorSubsystem elevator;
    private final VelocityMotorSubsystem<MotorInputsAutoLogged, MotorIO> flywheel;
    private final PositionMotorSubsystem<MotorInputsAutoLogged, MotorIO, Angle> intakePivot;
    private final BeamBreak beamBreak;

    public RobotContainer() {
        MotorIO elevatorIO;
        MotorIO flywheelIO;
        MotorIO intakePivotIO;
        BeamBreakIO beamBreakIO;

        if (Logger.hasReplaySource()) {
            elevatorIO = new MotorIO() {};
            flywheelIO = new MotorIO() {};
            intakePivotIO = new MotorIO() {};
            beamBreakIO = new BeamBreakIO() {};
        } else if (RobotBase.isReal()) {
            elevatorIO = new MotorIOTalonFX(ElevatorConfig.CONFIG);
            flywheelIO = new MotorIOTalonFX(FlywheelConfig.CONFIG);
            intakePivotIO = new MotorIOTalonFX(IntakePivotConfig.CONFIG);
            beamBreakIO = new BeamBreakIOAnalog(0);
        } else {
            elevatorIO = new MotorIOSim(ElevatorConfig.CONFIG);
            flywheelIO = new MotorIOSim(FlywheelConfig.CONFIG);
            intakePivotIO = new MotorIOSim(IntakePivotConfig.CONFIG);
            beamBreakIO = new BeamBreakIOSim();
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

        configureBindings();
    }

    public void robotPeriodic() {}

    private void configureBindings() {
        driver.a().onTrue(intakePivot.runPosition(Degrees.of(180)));
        driver.b().onTrue(intakePivot.runPosition(Degrees.of(90)));
        driver.y().onTrue(intakePivot.runPosition(Degrees.of(0)));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
