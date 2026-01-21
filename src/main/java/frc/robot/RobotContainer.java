// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.Configs.HoodConfig;
import frc.robot.subsystems.Configs.HoodParamsNT;
import frc.robot.subsystems.Configs.IndexerConfig;
import frc.robot.subsystems.Configs.IndexerParamsNT;
import frc.robot.subsystems.Configs.IntakerConfig;
import frc.robot.subsystems.Configs.IntakerParamsNT;
import frc.robot.subsystems.Configs.ShooterConfig;
import frc.robot.subsystems.Configs.ShooterParamsNT;
import frc.robot.subsystems.Configs.SwerveMK5Config;
import frc.robot.subsystems.Configs.TurretConfig;
import frc.robot.subsystems.Configs.TurretVelParamsNT;
import frc.robot.subsystems.ShootingSubsystem.ShootingParametersTable;
import frc.robot.subsystems.ShootingSubsystem.TurretSubsystem;
import frc.robot.subsystems.ShootingSubsystem.TurretSubsystem.TurretMode;
import lib.ironpulse.command.SysIdCommand;
import lib.ironpulse.command.VisualizeProjectileShot;
import lib.ironpulse.io.CANCoderIOCANCoder;
import lib.ironpulse.io.CANCoderIOSim;
import lib.ironpulse.io.MotorIOSim;
import lib.ironpulse.io.MotorIOTalonFX;
import lib.ironpulse.io.MotorInputsAutoLogged;
import lib.ironpulse.math.rbd.TransformRecorder;
import lib.ironpulse.subsystem.position.PositionMotorSubsystem;
import lib.ironpulse.subsystem.velocity.VelocityMotorSubsystem;
import lib.ironpulse.swerve.Swerve;
import lib.ironpulse.swerve.SwerveCommands;
import lib.ironpulse.swerve.mk5n.ImuIOPigeon;
import lib.ironpulse.swerve.mk5n.SwerveModuleIOMK5N;
import lib.ironpulse.swerve.sim.ImuIOSim;
import lib.ironpulse.swerve.sim.SwerveModuleIOSimpleSim;
import lib.ironpulse.utils.PhoenixUtils;
import lib.ntext.NTParameterRegistry;

@SuppressWarnings({"unused","rawtypes"})
public class RobotContainer {
    private final CommandXboxController driver = new CommandXboxController(0);
    private final ShootingParametersTable shootingParametersTable = new ShootingParametersTable();
    private final Swerve swerve;
    private final TurretSubsystem turret;
    private final VelocityMotorSubsystem shooter;
    private final VelocityMotorSubsystem indexer;
    private final PositionMotorSubsystem hood;
    private final CANCoderIOSim encoderG1Sim = new CANCoderIOSim();
    private final CANCoderIOSim encoderG2Sim = new CANCoderIOSim();

@SuppressWarnings("unchecked")
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
                turret =
                new TurretSubsystem(
                        TurretConfig.TURRET_CONFIG,
                        new MotorInputsAutoLogged(),
                        new MotorIOTalonFX(TurretConfig.TURRET_CONFIG),
                        new CANCoderIOCANCoder(
                                TurretConfig.TURRET_ENCODER_G1_ID,
                                TurretConfig.TURRET_ENCODER_G1_OFFSET,
                                false),
                        new CANCoderIOCANCoder(
                                TurretConfig.TURRET_ENCODER_G2_ID,
                                TurretConfig.TURRET_ENCODER_G2_OFFSET,
                                false),
                        TurretVelParamsNT.asVelocityParamSources());
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
                hood =
                        new PositionMotorSubsystem(
                                HoodConfig.HOOD_CONFIG,
                                new MotorInputsAutoLogged(),
                                new MotorIOTalonFX(HoodConfig.HOOD_CONFIG),
                                HoodParamsNT.asPositionParamSources(),
                                Degrees.of(0),
                                Degrees.of(360));


        } else {
            swerve =
                    new Swerve(
                            SwerveMK5Config.kSimConfig,
                            new ImuIOSim(),
                            new SwerveModuleIOSimpleSim(SwerveMK5Config.kSimConfig, 0),
                            new SwerveModuleIOSimpleSim(SwerveMK5Config.kSimConfig, 1),
                            new SwerveModuleIOSimpleSim(SwerveMK5Config.kSimConfig, 2),
                            new SwerveModuleIOSimpleSim(SwerveMK5Config.kSimConfig, 3));
        turret =
                            new TurretSubsystem(
                                    TurretConfig.TURRET_CONFIG,
                                    new MotorInputsAutoLogged(),
                                    new MotorIOSim(TurretConfig.TURRET_CONFIG),
                                    encoderG1Sim,
                                    encoderG2Sim,
                                    TurretVelParamsNT.asVelocityParamSources());
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
        hood =
                        new PositionMotorSubsystem(
                                HoodConfig.HOOD_CONFIG,
                                new MotorInputsAutoLogged(),
                                new MotorIOSim(HoodConfig.HOOD_CONFIG),
                                HoodParamsNT.asPositionParamSources(),
                                Degrees.of(0),
                                Degrees.of(360));
        }
        configureBindings();
        swerve.setDefaultCommand(
                SwerveCommands.driveWithJoystick(
                        swerve,
                        () -> -driver.getLeftY(),
                        () -> -driver.getLeftX(),
                        () -> -driver.getRightX(),
                        RobotStateRecorder::getPoseDriverRobotCurrent,
                        MetersPerSecond.of(0.04),
                        DegreesPerSecond.of(3.0)));

    }

    public void robotPeriodic() {
        // update IO inputs
        PhoenixUtils.refreshAll();
        // update NTparameters
        NTParameterRegistry.refresh();
        shootingParametersTable.updateFromNT();
        // update RobotStateRecorder
        var now = Seconds.of(Timer.getTimestamp());
        RobotStateRecorder.getInstance()
                .putTransform(
                        swerve.getEstimatedPose(),
                        now,
                        TransformRecorder.kFrameWorld,
                        TransformRecorder.kFrameRobot);

        RobotStateRecorder.getInstance()
                .putTransform(
                        new Pose3d(
                                RobotStateRecorder.kRobotToTurret,
                                new Rotation3d(0.0, 0.0, turret.getPosition().in(Radians))),
                        now,
                        TransformRecorder.kFrameRobot,
                        RobotStateRecorder.kFrameTurret);

        RobotStateRecorder.putVelocityRobot(now, swerve.getChassisSpeeds());
        RobotStateRecorder.periodic();
    }

    private void configureBindings() {
        driver.a()
                .whileTrue(
                        new VisualizeProjectileShot(
                                        RobotStateRecorder::getPoseWorldRobotCurrent,
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

        driver.povUp().onTrue(turret.setTurretPoseWorld(() -> Degrees.of(0), TurretMode.SEEKING));
        driver.povRight().onTrue(turret.setTurretPoseWorld(() -> Degrees.of(90), TurretMode.SEEKING));
        driver.povDown().onTrue(turret.setTurretPoseWorld(() -> Degrees.of(180), TurretMode.TRACKING));
        driver.povLeft().onTrue(turret.setTurretPoseWorld(() -> Degrees.of(270), TurretMode.TRACKING));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
