// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.*;
import frc.robot.subsystems.shooter.ShooterConfig;
import frc.robot.subsystems.shooter.ShootingParametersTable;
import lib.ironpulse.command.VisualizeProjectileShot;
import lib.ironpulse.math.rbd.TransformRecorder;
import lib.ironpulse.swerve.Swerve;
import lib.ironpulse.swerve.SwerveCommands;
import lib.ironpulse.swerve.mk5n.ImuIOPigeon;
import lib.ironpulse.swerve.mk5n.SwerveModuleIOMK5N;
import lib.ironpulse.swerve.sim.ImuIOSim;
import lib.ironpulse.swerve.sim.SwerveModuleIOSimpleSim;
import lib.ironpulse.utils.PhoenixUtils;
import lib.ntext.NTParameterRegistry;

public class RobotContainer {
    private final CommandXboxController driver = new CommandXboxController(0);
    private final ShootingParametersTable shootingParametersTable = new ShootingParametersTable();
    private final ShooterConfig shooterConfig = new ShooterConfig();
    private final Swerve swerve;

    public RobotContainer() {

        if (RobotBase.isReal()) {
            swerve =
                    new Swerve(
                            SwerveMK5Config.kRealConfig,
                            new ImuIOPigeon(SwerveMK5Config.kRealConfig),
                            new SwerveModuleIOMK5N(SwerveMK5Config.kRealConfig, 0),
                            new SwerveModuleIOMK5N(SwerveMK5Config.kRealConfig, 1),
                            new SwerveModuleIOMK5N(SwerveMK5Config.kRealConfig, 2),
                            new SwerveModuleIOMK5N(SwerveMK5Config.kRealConfig, 3));
        } else {
            swerve =
                    new Swerve(
                            SwerveMK5Config.kSimConfig,
                            new ImuIOSim(),
                            new SwerveModuleIOSimpleSim(SwerveMK5Config.kSimConfig, 0),
                            new SwerveModuleIOSimpleSim(SwerveMK5Config.kSimConfig, 1),
                            new SwerveModuleIOSimpleSim(SwerveMK5Config.kSimConfig, 2),
                            new SwerveModuleIOSimpleSim(SwerveMK5Config.kSimConfig, 3));
        }
        configureBindings();
        swerve.setDefaultCommand(
                SwerveCommands.driveWithJoystick(
                        swerve,
                        () -> -driver.getLeftY(),
                        () -> -driver.getLeftX(),
                        // () -> 0.0,
                        () -> driver.getRightX(),
                        RobotStateRecorder::getPoseDriverRobotCurrent,
                        // () -> new Pose3d(),
                        MetersPerSecond.of(0.04),
                        DegreesPerSecond.of(3.0)));
    }

    public void robotPeriodic() {
        PhoenixUtils.refreshAll();
        NTParameterRegistry.refresh();
        shootingParametersTable.updateFromNT();
        var now = Seconds.of(Timer.getTimestamp());
        RobotStateRecorder.getInstance()
                .putTransform(
                        swerve.getEstimatedPose(),
                        now,
                        TransformRecorder.kFrameWorld,
                        TransformRecorder.kFrameRobot);
        RobotStateRecorder.putVelocityRobot(now, swerve.getChassisSpeeds());
        RobotStateRecorder.periodic();
    }

    private void configureBindings() {
        driver.a()
                .whileTrue(
                        new VisualizeProjectileShot(
                                        () -> RobotStateRecorder.getPoseWorldRobotCurrent(),
                                        () ->
                                                RobotStateRecorder.getPoseWorldRobotCurrent()
                                                        .getRotation()
                                                        .toRotation2d(),
                                        () -> new Rotation2d(45),
                                        () -> 10,
                                        () ->
                                                RobotStateRecorder.getVelocityWorldRobotCurrent()
                                                        .getTranslation(),
                                        true)
                                .repeatedly());
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
