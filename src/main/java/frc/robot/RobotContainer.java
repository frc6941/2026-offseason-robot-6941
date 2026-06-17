// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Configs.IntakeConfig;
import frc.robot.subsystems.Configs.IntakerExtensionParamsNT;
import frc.robot.subsystems.Configs.IntakerRollerParamsNT;
import frc.robot.subsystems.Configs.SwerveMK5Config;
import frc.robot.subsystems.IntakerSubsystem;
import lib.ironpulse.io.MotorIO;
import lib.ironpulse.io.MotorIOSim;
import lib.ironpulse.io.MotorIOTalonFX;
import lib.ironpulse.io.MotorInputsAutoLogged;
import lib.ironpulse.subsystem.position.PositionMotorSubsystem;
import lib.ironpulse.subsystem.velocity.VelocityMotorSubsystem;
import lib.ironpulse.swerve.Swerve;
import lib.ironpulse.swerve.SwerveCommands;
import lib.ironpulse.swerve.mk5n.ImuIOPigeon;
import lib.ironpulse.swerve.mk5n.SwerveModuleIOMK5N;
import lib.ironpulse.swerve.sim.ImuIOSim;
import lib.ironpulse.swerve.sim.SwerveModuleIOSimpleSim;

public class RobotContainer {
    private static final boolean HAS_INTAKER_ROLLER_IO = true;
    private static final boolean HAS_INTAKER_EXTENSION_IO = true;
    private static final boolean HAS_SWERVE_IO = true;

    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);

    private final Swerve swerve;
    private final IntakerSubsystem intake;

    public RobotContainer() {
        final boolean isReal = RobotBase.isReal();

        swerve = buildSwerve(isReal && HAS_SWERVE_IO);

        var intakerRoller = buildIntakerRoller(isReal && HAS_INTAKER_ROLLER_IO);
        var intakerExtension = buildIntakerExtension(isReal && HAS_INTAKER_EXTENSION_IO);

        intake = new IntakerSubsystem(intakerRoller, intakerExtension);

        // set default commands
        swerve.setDefaultCommand(
                SwerveCommands.driveWithJoystick(
                        swerve,
                        () -> -driver.getLeftY(),
                        () -> -driver.getLeftX(),
                        () -> -driver.getRightX(),
                        () -> new Pose3d(),
                        MetersPerSecond.of(0.04),
                        DegreesPerSecond.of(3.0)));

        intake.setDefaultCommand();

        // basic bindings for intake
        driver.leftBumper().onTrue(intake.toggleIntake());
        driver.leftTrigger().whileTrue(intake.runFeed());
        driver.povDown().onTrue(intake.runRetract());
        operator.leftBumper().whileTrue(intake.runExtendedReverse());
    }

    private Swerve buildSwerve(boolean isReal) {
        return new Swerve(
                isReal ? SwerveMK5Config.kRealConfig : SwerveMK5Config.kSimConfig,
                isReal
                        ? new ImuIOPigeon(SwerveMK5Config.kRealConfig, SwerveMK5Config.pigeonConfig)
                        : new ImuIOSim(),
                isReal
                        ? new SwerveModuleIOMK5N(SwerveMK5Config.kRealConfig, 0)
                        : new SwerveModuleIOSimpleSim(SwerveMK5Config.kSimConfig, 0),
                isReal
                        ? new SwerveModuleIOMK5N(SwerveMK5Config.kRealConfig, 1)
                        : new SwerveModuleIOSimpleSim(SwerveMK5Config.kSimConfig, 1),
                isReal
                        ? new SwerveModuleIOMK5N(SwerveMK5Config.kRealConfig, 2)
                        : new SwerveModuleIOSimpleSim(SwerveMK5Config.kSimConfig, 2),
                isReal
                        ? new SwerveModuleIOMK5N(SwerveMK5Config.kRealConfig, 3)
                        : new SwerveModuleIOSimpleSim(SwerveMK5Config.kSimConfig, 3));
    }

    private VelocityMotorSubsystem<MotorInputsAutoLogged, MotorIO> buildIntakerRoller(
            boolean isReal) {
        return new VelocityMotorSubsystem<>(
                IntakeConfig.INTAKER_ROLLER_CONFIG,
                new MotorInputsAutoLogged(),
                isReal
                        ? new MotorIOTalonFX(IntakeConfig.INTAKER_ROLLER_CONFIG)
                        : new MotorIOSim(IntakeConfig.INTAKER_ROLLER_CONFIG),
                IntakerRollerParamsNT.asVelocityParamSources());
    }

    private PositionMotorSubsystem<
                    MotorInputsAutoLogged, MotorIO, edu.wpi.first.units.measure.Distance>
            buildIntakerExtension(boolean isReal) {
        return new PositionMotorSubsystem<>(
                IntakeConfig.INTAKER_EXTENSION_CONFIG,
                new MotorInputsAutoLogged(),
                isReal
                        ? new MotorIOTalonFX(IntakeConfig.INTAKER_EXTENSION_CONFIG)
                        : new MotorIOSim(IntakeConfig.INTAKER_EXTENSION_CONFIG),
                IntakerExtensionParamsNT.asPositionParamSources(),
                Meters.of(0),
                IntakeConfig.INTAKE_EXTENSION_METERS_PER_ROTATION);
    }

    public void robotPeriodic() {
        // Update NTParameters for Intaker/Swerve subsystems
        lib.ntext.NTParameterRegistry.refresh();
    }

    public void setThrottle(boolean enabled) {
        // Throttle management for subsystems; placeholder for future expansion
    }

    public Command getAutonomousCommand() {
        return null;
    }
}
