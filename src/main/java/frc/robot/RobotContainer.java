// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveConstants;
import lib.ironpulse.swerve.Swerve;
import lib.ironpulse.swerve.sim.ImuIOSim;
import lib.ironpulse.swerve.sim.SwerveModuleIOSimpleSim;
import lib.ironpulse.swerve.sjtu6.ImuIOPigeon;
import lib.ironpulse.swerve.sjtu6.SwerveModuleIOSJTU6;

import static edu.wpi.first.units.Units.Meters;

public class RobotContainer {
    // private final IntakePivotSubsystem intakePivot = new IntakePivotSubsystem();
    // private final FlywheelSubsystem flywheelSubsystem = new FlywheelSubsystem();
    private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    private final CommandXboxController driver = new CommandXboxController(0);
    private Swerve swerve;

    public RobotContainer() {
        if (RobotBase.isReal()) {
            swerve = new Swerve(
                    SwerveConstants.kRealConfig,
                    new ImuIOPigeon(SwerveConstants.kRealConfig),
                    new SwerveModuleIOSJTU6(SwerveConstants.kRealConfig, 0),
                    new SwerveModuleIOSJTU6(SwerveConstants.kRealConfig, 1),
                    new SwerveModuleIOSJTU6(SwerveConstants.kRealConfig, 2),
                    new SwerveModuleIOSJTU6(SwerveConstants.kRealConfig, 3));
        } else {
            swerve = new Swerve(
                    SwerveConstants.kSimConfig,
                    new ImuIOSim(),
                    new SwerveModuleIOSimpleSim(SwerveConstants.kSimConfig, 0),
                    new SwerveModuleIOSimpleSim(SwerveConstants.kSimConfig, 1),
                    new SwerveModuleIOSimpleSim(SwerveConstants.kSimConfig, 2),
                    new SwerveModuleIOSimpleSim(SwerveConstants.kSimConfig, 3));
        }
        configureBindings();
    }

    public void robotPeriodic() {
        // reserved for vision fusion
    }

    private void configureBindings() {

        driver.a().onTrue(elevatorSubsystem.runVoltage(2));
        driver.b().onTrue(elevatorSubsystem.runVoltage(() -> 2));
        driver.leftBumper().onTrue(elevatorSubsystem.runStop());

        driver.x().onTrue(elevatorSubsystem.runMotionMagic(Meters.of(1.2)));
        driver.y().onTrue(elevatorSubsystem.runMotionMagic(Meters.of(0.2)));

        driver.start().whileTrue(elevatorSubsystem.zeroCommand());
        // driver.x().onTrue(Commands.runOnce(() ->
        // intakePivot.setMotionMagicSetpoint(Degrees.of(0))));
        // driver.y().onTrue(Commands.runOnce(() ->
        // intakePivot.setPositionSetpoint(Degrees.of(40\[]\[].0))));

    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
