// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Configs.*;
import frc.robot.subsystems.Configs.HoodConfig;
import frc.robot.subsystems.Configs.HoodParamsNT;
import frc.robot.subsystems.Configs.IdxConfig;
import frc.robot.subsystems.Configs.ShooterConfig;
import frc.robot.subsystems.Configs.ShooterParamsNT;
import frc.robot.subsystems.Configs.SpindexerParamsNT;
import frc.robot.subsystems.Configs.SwerveMK5Config;
import frc.robot.subsystems.Configs.TurretConfig;
import frc.robot.subsystems.Configs.TurretVelParamsNT;
import frc.robot.subsystems.ShootingSubsystem.ShotCalculator;
import frc.robot.subsystems.ShootingSubsystem.TurretSubsystem;
import frc.robot.subsystems.ShootingSubsystem.TurretSubsystem.TurretMode;
import lib.ironpulse.io.CANCoderIOSim;
import lib.ironpulse.io.MotorIO;
import lib.ironpulse.io.MotorIOSim;
import lib.ironpulse.io.MotorIOTalonFX;
import lib.ironpulse.io.MotorInputsAutoLogged;
import lib.ironpulse.math.rbd.TransformRecorder;
import lib.ironpulse.subsystem.position.PositionMotorSubsystem;
import lib.ironpulse.subsystem.velocity.VelocityMotorSubsystem;
import lib.ironpulse.swerve.Swerve;
import lib.ironpulse.swerve.SwerveCommands;
import lib.ironpulse.swerve.sim.ImuIOSim;
import lib.ironpulse.swerve.sim.SwerveModuleIOSimpleSim;
import lib.ironpulse.utils.AllianceFlipUtil;
import lib.ironpulse.utils.PhoenixUtils;
import lib.ntext.NTParameterRegistry;

@SuppressWarnings("rawtypes")
public class RobotContainer {
    private static final boolean HAS_TURRET_IO = true;
    private static final boolean HAS_SHOOTER_IO = false;
    private static final boolean HAS_HOOD_IO = false;
    private static final boolean HAS_IDX_IO = false;
    // private final LimelightSubsystem limelightSubsystem;
    // private final IntakerSubsystem intakerSubsystem;
    private final CommandXboxController driver = new CommandXboxController(0);
    private final ShotCalculator shotCalculator = new ShotCalculator();
    private final Swerve swerve;
    private final TurretSubsystem turret;
    private final VelocityMotorSubsystem<MotorInputsAutoLogged, MotorIO> shooter;
    private final VelocityMotorSubsystem<MotorInputsAutoLogged, MotorIO> spindexer;
    private final PositionMotorSubsystem<MotorInputsAutoLogged, MotorIO, Angle> hood;
    // private final ShootingSuperstructure shootingSuperstructure;
    //     private final VelocityMotorSubsystem intakerRoller;
    //     private final PositionMotorSubsystem intakerExtension;
    private final CANCoderIOSim encoderG1Sim = new CANCoderIOSim();
    private final CANCoderIOSim encoderG2Sim = new CANCoderIOSim();

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
            swerve =
                    new Swerve(
                            SwerveMK5Config.kSimConfig,
                            new ImuIOSim(),
                            new SwerveModuleIOSimpleSim(SwerveMK5Config.kSimConfig, 0),
                            new SwerveModuleIOSimpleSim(SwerveMK5Config.kSimConfig, 1),
                            new SwerveModuleIOSimpleSim(SwerveMK5Config.kSimConfig, 2),
                            new SwerveModuleIOSimpleSim(SwerveMK5Config.kSimConfig, 3));
            //     limelightSubsystem =
            //             new LimelightSubsystem(
            //                     RobotConstants.LimelightConstants.limelightSubsystemConfig,
            //                     swerve,
            //                     new LimelightIOReal(
            //                             RobotConstants.LimelightConstants.limelight1Config,
            //                             () ->
            //                                     RobotStateRecorder.getPoseWorldRobotCurrent()
            //                                             .toPose2d()
            //                                             .getRotation()
            //                                             .getDegrees(),
            //                             () -> {
            //                                 // angular velocity > 360 deg per second
            //                                 return
            // RobotStateRecorder.getVelocityWorldRobotCurrent()
            //                                                 .getRotation()
            //                                                 .getDegrees()
            //                                         > 360;
            //                             }));
            turret =
                    new TurretSubsystem(
                            TurretConfig.TURRET_CONFIG,
                            new MotorInputsAutoLogged(),
                            HAS_TURRET_IO
                                    ? new MotorIOTalonFX(TurretConfig.TURRET_CONFIG)
                                    : new MotorIOSim(TurretConfig.TURRET_CONFIG),
                            HAS_TURRET_IO
                                    ? encoderG1Sim
                                    //     ? new CANCoderIOCANCoder(
                                    //             TurretConfig.TURRET_ENCODER_G1_ID,
                                    //             TurretConfig.TURRET_ENCODER_G1_OFFSET,
                                    //             false)
                                    : encoderG1Sim,
                            HAS_TURRET_IO
                                    ? encoderG2Sim
                                    //     ? new CANCoderIOCANCoder(
                                    //             TurretConfig.TURRET_ENCODER_G2_ID,
                                    //             TurretConfig.TURRET_ENCODER_G2_OFFSET,
                                    //             false)
                                    : encoderG2Sim,
                            TurretVelParamsNT.asVelocityParamSources());
            shooter =
                    new VelocityMotorSubsystem<>(
                            ShooterConfig.SHOOTER_CONFIG,
                            new MotorInputsAutoLogged(),
                            HAS_SHOOTER_IO
                                    ? new MotorIOTalonFX(ShooterConfig.SHOOTER_CONFIG)
                                    : new MotorIOSim(ShooterConfig.SHOOTER_CONFIG),
                            ShooterParamsNT.asVelocityParamSources());
            spindexer =
                    new VelocityMotorSubsystem<>(
                            IdxConfig.SPINDEXER_CFG,
                            new MotorInputsAutoLogged(),
                            HAS_IDX_IO
                                    ? new MotorIOTalonFX(IdxConfig.SPINDEXER_CFG)
                                    : new MotorIOSim(IdxConfig.SPINDEXER_CFG),
                            SpindexerParamsNT.asVelocityParamSources());
            hood =
                    new PositionMotorSubsystem<>(
                            HoodConfig.HOOD_CONFIG,
                            new MotorInputsAutoLogged(),
                            HAS_HOOD_IO
                                    ? new MotorIOTalonFX(HoodConfig.HOOD_CONFIG)
                                    : new MotorIOSim(HoodConfig.HOOD_CONFIG),
                            HoodParamsNT.asPositionParamSources(),
                            Degrees.of(0),
                            Degrees.of(360.0 / HoodConfig.HOOD_CONFIG.SensorToMechanismRatio));
            //     intakerRoller =
            //             new VelocityMotorSubsystem(
            //                     IntakerRollerConfig.INTAKER_ROLLER_CONFIG,
            //                     new MotorInputsAutoLogged(),
            //                     new MotorIOTalonFX(IntakerRollerConfig.INTAKER_ROLLER_CONFIG),
            //                     IntakerRollerParamsNT.asVelocityParamSources());
            //     intakerExtension =
            //             new PositionMotorSubsystem(
            //                     IntakerExtensionConfig.INTAKER_EXTENSION_CONFIG,
            //                     new MotorInputsAutoLogged(),
            //                     new
            // MotorIOTalonFX(IntakerExtensionConfig.INTAKER_EXTENSION_CONFIG),
            //                     IntakerExtensionParamsNT.asPositionParamSources(),
            //                     Meters.of(0),
            //                     Meters.of(0.00942 * 11.0));

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
            //     intakerRoller =
            //             new VelocityMotorSubsystem(
            //                     IntakerRollerConfig.INTAKER_ROLLER_CONFIG,
            //                     new MotorInputsAutoLogged(),
            //                     new MotorIOSim(IntakerRollerConfig.INTAKER_ROLLER_CONFIG),
            //                     IntakerRollerParamsNT.asVelocityParamSources());
            shooter =
                    new VelocityMotorSubsystem<>(
                            ShooterConfig.SHOOTER_CONFIG,
                            new MotorInputsAutoLogged(),
                            new MotorIOSim(ShooterConfig.SHOOTER_CONFIG),
                            ShooterParamsNT.asVelocityParamSources());
            spindexer =
                    new VelocityMotorSubsystem<>(
                            IdxConfig.SPINDEXER_CFG,
                            new MotorInputsAutoLogged(),
                            new MotorIOSim(IdxConfig.SPINDEXER_CFG),
                            SpindexerParamsNT.asVelocityParamSources());
            hood =
                    new PositionMotorSubsystem<>(
                            HoodConfig.HOOD_CONFIG,
                            new MotorInputsAutoLogged(),
                            new MotorIOSim(HoodConfig.HOOD_CONFIG),
                            HoodParamsNT.asPositionParamSources(),
                            Degrees.of(0),
                            Degrees.of(360));
            //     intakerExtension =
            //             new PositionMotorSubsystem(
            //                     IntakerExtensionConfig.INTAKER_EXTENSION_CONFIG,
            //                     new MotorInputsAutoLogged(),
            //                     new MotorIOSim(IntakerExtensionConfig.INTAKER_EXTENSION_CONFIG),
            //                     IntakerExtensionParamsNT.asPositionParamSources(),
            //                     Meters.of(0),
            //                     Millimeters.of(9.42 * 11.0));
            // TODO: limelight simulation
            //     limelightSubsystem =
            //             new LimelightSubsystem(
            //                     RobotConstants.LimelightConstants.limelightSubsystemConfig,
            // swerve);
        }

        // shotCalculator.initialize(
        //         Map.of(
        //                 ShotCalculator.TargetMode.GOAL,
        //                 Filesystem.getDeployDirectory().toPath().resolve("results_GOAL.json")));
        // shootingSuperstructure = new ShootingSuperstructure(turret, hood, shooter, spindexer);
        // intakerSubsystem = new IntakerSubsystem(intakerRoller, intakerExtension);
        configureBindings();
        // shootingSuperstructure.setDefaultCommand();
        swerve.setDefaultCommand(
                SwerveCommands.driveWithJoystick(
                        swerve,
                        () -> -driver.getLeftY(),
                        () -> -driver.getLeftX(),
                        () -> -driver.getRightX(),
                        RobotStateRecorder::getPoseDriverRobotCurrent,
                        MetersPerSecond.of(0.04),
                        DegreesPerSecond.of(3.0)));
        turret.setDefaultCommand(turret.runTurretTargetLoop());
    }

    public void robotPeriodic() {
        // update IO inputs
        PhoenixUtils.refreshAll();
        // update NTparameters
        NTParameterRegistry.refresh();
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
                                RobotStateRecorder.kRobotToShot,
                                new Rotation3d(
                                        0.0,
                                        hood.getCurrPos().in(Radians),
                                        turret.getPosition().in(Radians))),
                        now,
                        TransformRecorder.kFrameRobot,
                        RobotStateRecorder.kFrameShot);

        RobotStateRecorder.putVelocityRobot(now, swerve.getChassisSpeeds());
        // RobotStateRecorder.setCurrentFrame(shootingSuperstructure.getCurrentFrame());
        RobotStateRecorder.periodic();
        //        LimelightHelpers.SetRobotOrientation(
        //                "limelight",
        //
        // RobotStateRecorder.getPoseWorldRobotCurrent().toPose2d().getRotation().getDegrees(),
        //
        // RobotStateRecorder.getVelocityWorldRobotCurrent().getRotation().getDegrees(),
        //                0,
        //                0,
        //                0,
        //                0);
        //        Logger.recordOutput(
        //                "Limelight/Pose",
        //                LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight").pose);
        //        swerve.addVisionMeasurement(
        //                new
        // Pose3d(LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight").pose),
        //                now.in(Seconds),
        //                VecBuilder.fill(0.1, 0.1, 0.3, 100.0));
    }

    private void configureBindings() {
        // SysIdCommand turretSysId = new SysIdCommand(turret);

        driver.a().onTrue(turret.setCurrentPosition(Degrees.zero()).ignoringDisable(true));
        driver.povUp().onTrue(turret.setTurretPoseWorld(() -> Degrees.of(0), TurretMode.SEEKING));
        driver.povLeft()
                .onTrue(turret.setTurretPoseWorld(() -> Degrees.of(135), TurretMode.SEEKING));
        driver.povDown()
                .onTrue(turret.setTurretPoseWorld(() -> Degrees.of(180), TurretMode.SEEKING));
        driver.povRight()
                .onTrue(turret.setTurretPoseWorld(() -> Degrees.of(-135), TurretMode.SEEKING));

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
        // driver.a().whileTrue(turretSysId.quasistatic(SysIdRoutine.Direction.kForward));
        // driver.b().whileTrue(turretSysId.quasistatic(SysIdRoutine.Direction.kReverse));
        // driver.x().whileTrue(turretSysId.dynamic(SysIdRoutine.Direction.kForward));
        // driver.y().whileTrue(turretSysId.dynamic(SysIdRoutine.Direction.kReverse));

        // driver.a()
        //         .whileTrue(
        //                 shootingSuperstructure
        //                         .runFrame(
        //                                 () ->
        //                                         shotCalculator.computeShotFrame(
        //                                                 ShotCalculator.TargetMode.GOAL),
        //                                 TurretMode.TRACKING)
        //                         .alongWith(
        //                                 Commands.run(
        //                                         () -> {
        //                                             if (shootingSuperstructure.readyToShoot()) {
        //                                                 var frame =
        //                                                         RobotStateRecorder
        //                                                                 .getCurrentFrame();
        //                                                 VisualizeProjectileShot.logPath(
        //                                                         RobotStateRecorder
        //
        // .getPoseWorldShotCurrent(),
        //                                                         Rotation2d.fromRadians(
        //                                                                 frame.turretAngleWorld()
        //                                                                         .in(Radians)),
        //                                                         Rotation2d.fromRadians(
        //                                                                 frame.hoodAngle()
        //                                                                         .in(Radians)),
        //                                                         frame.muzzleSpeed()
        //                                                                 .in(MetersPerSecond),
        //                                                         RobotStateRecorder
        //
        // .getVelocityWorldRobotCurrent()
        //                                                                 .getTranslation(),
        //                                                         true,
        //                                                         "Valid");
        //                                             } else {
        //                                                 var frame =
        //                                                         RobotStateRecorder
        //                                                                 .getCurrentFrame();
        //                                                 VisualizeProjectileShot.logPath(
        //                                                         RobotStateRecorder
        //
        // .getPoseWorldShotCurrent(),
        //                                                         Rotation2d.fromRadians(
        //                                                                 frame.turretAngleWorld()
        //                                                                         .in(Radians)),
        //                                                         Rotation2d.fromRadians(
        //                                                                 frame.hoodAngle()
        //                                                                         .in(Radians)),
        //                                                         frame.muzzleSpeed()
        //                                                                 .in(MetersPerSecond),
        //                                                         RobotStateRecorder
        //
        // .getVelocityWorldRobotCurrent()
        //                                                                 .getTranslation(),
        //                                                         true,
        //                                                         "Invalid");
        //                                             }
        //                                         })));
        // driver.povUp().onTrue(turret.setTurretPoseWorld(() -> Degrees.of(0),
        // TurretMode.SEEKING));
        // driver.povRight()
        //         .onTrue(turret.setTurretPoseWorld(() -> Degrees.of(90), TurretMode.SEEKING));
        // driver.povDown()
        //         .onTrue(turret.setTurretPoseWorld(() -> Degrees.of(180), TurretMode.TRACKING));
        // driver.povLeft()
        //         .onTrue(turret.setTurretPoseWorld(() -> Degrees.of(270), TurretMode.TRACKING));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
