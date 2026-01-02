// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.SwerveConstants;
import frc.robot.util.AllianceFlipUtil;
import lib.ironpulse.math.rbd.TransformRecorder;
import lib.ironpulse.swerve.Swerve;
import lib.ironpulse.swerve.SwerveCommands;
import lib.ironpulse.swerve.mk5n.ImuIOPigeon;
import lib.ironpulse.swerve.mk5n.SwerveModuleIOMK5N;
import lib.ironpulse.swerve.sim.ImuIOSim;
import lib.ironpulse.swerve.sim.SwerveModuleIOSimpleSim;
import lib.ironpulse.utils.LimelightHelpers;
import org.littletonrobotics.junction.Logger;

public class RobotContainer {
    private Swerve swerve;
    // private final IntakePivotSubsystem intakePivot = new IntakePivotSubsystem();
    private final CommandXboxController driver = new CommandXboxController(0);

    public RobotContainer() {
        if (RobotBase.isReal()) {
            swerve =
                    new Swerve(
                            SwerveConstants.kRealConfig,
                            new ImuIOPigeon(SwerveConstants.kRealConfig),
                            new SwerveModuleIOMK5N(SwerveConstants.kRealConfig, 0),
                            new SwerveModuleIOMK5N(SwerveConstants.kRealConfig, 1),
                            new SwerveModuleIOMK5N(SwerveConstants.kRealConfig, 2),
                            new SwerveModuleIOMK5N(SwerveConstants.kRealConfig, 3));
        } else {
            swerve =
                    new Swerve(
                            SwerveConstants.kSimConfig,
                            new ImuIOSim(),
                            new SwerveModuleIOSimpleSim(SwerveConstants.kSimConfig, 0),
                            new SwerveModuleIOSimpleSim(SwerveConstants.kSimConfig, 1),
                            new SwerveModuleIOSimpleSim(SwerveConstants.kSimConfig, 2),
                            new SwerveModuleIOSimpleSim(SwerveConstants.kSimConfig, 3));
        }
        configureBindings();
        LimelightHelpers.SetIMUMode("limelight", 1);
    }

    public void robotPeriodic() {
        var now = Seconds.of(Timer.getTimestamp());
        RobotStateRecorder.getInstance()
                .putTransform(
                        swerve.getEstimatedPose(),
                        now,
                        TransformRecorder.kFrameWorld,
                        TransformRecorder.kFrameRobot);
        RobotStateRecorder.putVelocityRobot(now, swerve.getChassisSpeeds());
        RobotStateRecorder.periodic();
        LimelightHelpers.SetRobotOrientation(
                "limelight",
                RobotStateRecorder.getPoseWorldRobotCurrent().toPose2d().getRotation().getDegrees(),
                RobotStateRecorder.getVelocityWorldRobotCurrent().getRotation().getDegrees(),
                0,
                0,
                0,
                0);
        Logger.recordOutput(
                "Limelight/Pose",
                LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight").pose);
        swerve.addVisionMeasurement(
                new Pose3d(LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight").pose),
                now.in(Seconds),
                VecBuilder.fill(0.1, 0.1, 0.3, 100.0));
    }

    private void configureBindings() {
        // Intake Pivot: A -> 0 degrees, B -> 100 degrees
        // driver.a().onTrue(Commands.runOnce(() ->
        // intakePivot.setPositionSetpoint(Degrees.of(0.0)), intakePivot));
        // driver.b().onTrue(Commands.runOnce(() ->
        // intakePivot.setPositionSetpoint(Degrees.of(40.0)), intakePivot));
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

        driver.start()
                .onTrue(
                        SwerveCommands.resetAngle(
                                        swerve,
                                        () ->
                                                AllianceFlipUtil.shouldFlip()
                                                        ? Rotation2d.kZero
                                                        : Rotation2d.k180deg)
                                .alongWith(
                                        Commands.runOnce(
                                                () -> {
                                                    RobotStateRecorder.getInstance()
                                                            .resetTransform(
                                                                    TransformRecorder.kFrameWorld,
                                                                    TransformRecorder.kFrameRobot);
                                                }))
                                .ignoringDisable(true));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
