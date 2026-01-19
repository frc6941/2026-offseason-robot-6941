// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.Configs.IndexerConfig;
import frc.robot.subsystems.Configs.IndexerParamsNT;
import frc.robot.subsystems.Configs.ShooterConfig;
import frc.robot.subsystems.Configs.ShooterParamsNT;
import frc.robot.subsystems.ShootingSubsystem.ShootingParametersTable;
import lib.ironpulse.command.SysIdCommand;
import lib.ironpulse.io.MotorIOSim;
import lib.ironpulse.io.MotorIOTalonFX;
import lib.ironpulse.io.MotorInputsAutoLogged;
import lib.ironpulse.subsystem.velocity.VelocityMotorSubsystem;
import lib.ironpulse.utils.PhoenixUtils;
import lib.ntext.NTParameterRegistry;

public class RobotContainer {
    private final CommandXboxController driver = new CommandXboxController(0);
    private final ShootingParametersTable shootingParametersTable = new ShootingParametersTable();
    // private final Swerve swerve;
    private final VelocityMotorSubsystem shooter;
    private final VelocityMotorSubsystem indexer;

    public RobotContainer() {

        if (RobotBase.isReal()) {
            //     swerve =
            //             new Swerve(
            //                     SwerveMK5Config.kRealConfig,
            //                     new ImuIOPigeon(SwerveMK5Config.kRealConfig),
            //                     new SwerveModuleIOMK5N(SwerveMK5Config.kRealConfig, 0),
            //                     new SwerveModuleIOMK5N(SwerveMK5Config.kRealConfig, 1),
            //                     new SwerveModuleIOMK5N(SwerveMK5Config.kRealConfig, 2),
            //                     new SwerveModuleIOMK5N(SwerveMK5Config.kRealConfig, 3));
            shooter =
                    new VelocityMotorSubsystem(
                            ShooterConfig.SHOOTER_CONFIG,
                            new MotorInputsAutoLogged(),
                            new MotorIOTalonFX(ShooterConfig.SHOOTER_CONFIG),
                            ShooterParamsNT.asVelocityParamSources());
            indexer =
                    new VelocityMotorSubsystem(
                            IndexerConfig.INDEXER_CONFIG,
                            new MotorInputsAutoLogged(),
                            new MotorIOTalonFX(IndexerConfig.INDEXER_CONFIG),
                            IndexerParamsNT.asVelocityParamSources());

        } else {
            //     swerve =
            //             new Swerve(
            //                     SwerveMK5Config.kSimConfig,
            //                     new ImuIOSim(),
            //                     new SwerveModuleIOSimpleSim(SwerveMK5Config.kSimConfig, 0),
            //                     new SwerveModuleIOSimpleSim(SwerveMK5Config.kSimConfig, 1),
            //                     new SwerveModuleIOSimpleSim(SwerveMK5Config.kSimConfig, 2),
            //                     new SwerveModuleIOSimpleSim(SwerveMK5Config.kSimConfig, 3));
            shooter =
                    new VelocityMotorSubsystem(
                            ShooterConfig.SHOOTER_CONFIG,
                            new MotorInputsAutoLogged(),
                            new MotorIOSim(ShooterConfig.SHOOTER_CONFIG),
                            ShooterParamsNT.asVelocityParamSources());
            indexer =
                    new VelocityMotorSubsystem(
                            IndexerConfig.INDEXER_CONFIG,
                            new MotorInputsAutoLogged(),
                            new MotorIOSim(IndexerConfig.INDEXER_CONFIG),
                            IndexerParamsNT.asVelocityParamSources());
        }
        configureBindings();
        // swerve.setDefaultCommand(
        //         SwerveCommands.driveWithJoystick(
        //                 swerve,
        //                 () -> -driver.getLeftY(),
        //                 () -> -driver.getLeftX(),
        //                 // () -> 0.0,
        //                 () -> driver.getRightX(),
        //                 RobotStateRecorder::getPoseDriverRobotCurrent,
        //                 // () -> new Pose3d(),
        //                 MetersPerSecond.of(0.04),
        //                 DegreesPerSecond.of(3.0)));
        shooter.setDefaultCommand(shooter.runStop());
        // shooter.runVelocity(RotationsPerSecond.of(ShooterParamsNT.idleVelRPS.getValue())));
    }

    public void robotPeriodic() {
        PhoenixUtils.refreshAll();
        NTParameterRegistry.refresh();
        shootingParametersTable.updateFromNT();
        var now = Seconds.of(Timer.getTimestamp());
        // RobotStateRecorder.getInstance()
        //         .putTransform(
        //                 swerve.getEstimatedPose(),
        //                 now,
        //                 TransformRecorder.kFrameWorld,
        //                 TransformRecorder.kFrameRobot);
        // RobotStateRecorder.putVelocityRobot(now, swerve.getChassisSpeeds());
        RobotStateRecorder.periodic();
    }

    private void configureBindings() {
        driver.leftBumper()
                .onTrue(
                        shooter.runVelocity(
                                        RotationsPerSecond.of(
                                                ShooterParamsNT.testVelRPS.getValue()))
                                .alongWith(
                                        indexer.runVelocity(
                                                RotationsPerSecond.of(
                                                        IndexerParamsNT.testVelRPS.getValue()))));
        driver.rightBumper().whileTrue(shooter.runDutyCycle(1).alongWith(indexer.runDutyCycle(1)));
        driver.a().onTrue(shooter.runStop().alongWith(indexer.runStop()));
        driver.povUp()
                .whileTrue(new SysIdCommand(shooter).quasistatic(SysIdRoutine.Direction.kForward));
        driver.povDown()
                .whileTrue(new SysIdCommand(shooter).quasistatic(SysIdRoutine.Direction.kReverse));
        driver.povLeft()
                .whileTrue(new SysIdCommand(shooter).dynamic(SysIdRoutine.Direction.kForward));
        driver.povRight()
                .whileTrue(new SysIdCommand(shooter).dynamic(SysIdRoutine.Direction.kReverse));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
