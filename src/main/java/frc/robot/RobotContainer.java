// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import static frc.robot.RobotConstants.LED_LENGTH;
import static frc.robot.RobotConstants.LED_PORT;
import static frc.robot.RobotConstants.ROBORIO_CAN_BUS;

import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.auto.AutoActions;
import frc.robot.auto.AutoFile;
import frc.robot.auto.AutoRoutines;
import frc.robot.subsystems.Configs.*;
import frc.robot.subsystems.IntakerSubsystem;
import frc.robot.subsystems.ShootingSubsystem.ShootingSuperstructure;
import frc.robot.subsystems.ShootingSubsystem.ShotCalculator;
import frc.robot.subsystems.ShootingSubsystem.ShotCalculator.TargetMode;
import frc.robot.subsystems.ShootingSubsystem.SpindexerSubsystem;
import frc.robot.subsystems.ShootingSubsystem.TurretSubsystem;
import frc.robot.utils.HubShiftUtil;
import java.nio.file.Path;
import java.util.Map;
import java.util.Optional;

import lib.ironpulse.command.SysIdCommand;
import lib.ironpulse.display.FieldView;
import lib.ironpulse.indicator.IndicatorIO.Patterns;
import lib.ironpulse.indicator.IndicatorIOARGB;
import lib.ironpulse.indicator.IndicatorIOSim;
import lib.ironpulse.indicator.IndicatorSubsystem;
import lib.ironpulse.io.CANCoderIOCANCoder;
import lib.ironpulse.io.CANCoderIOSim;
import lib.ironpulse.io.MotorIO;
import lib.ironpulse.io.MotorIOSim;
import lib.ironpulse.io.MotorIOTalonFX;
import lib.ironpulse.io.MotorInputsAutoLogged;
import lib.ironpulse.limelight.LimelightIOReal;
import lib.ironpulse.limelight.LimelightSubsystem;
import lib.ironpulse.math.rbd.TransformRecorder;
import lib.ironpulse.subsystem.position.PositionMotorSubsystem;
import lib.ironpulse.subsystem.velocity.VelocityMotorSubsystem;
import lib.ironpulse.swerve.Swerve;
import lib.ironpulse.swerve.SwerveCommands;
import lib.ironpulse.swerve.SwerveModuleLimit;
import lib.ironpulse.swerve.mk5n.ImuIOPigeon;
import lib.ironpulse.swerve.mk5n.SwerveModuleIOMK5N;
import lib.ironpulse.swerve.sim.ImuIOSim;
import lib.ironpulse.swerve.sim.SwerveModuleIOSimpleSim;
import lib.ironpulse.utils.AllianceFlipUtil;
import lib.ironpulse.utils.PhoenixUtils;
import lib.ntext.NTParameterRegistry;
import lombok.SneakyThrows;
import org.littletonrobotics.junction.Logger;

public class RobotContainer {
    private static final boolean HAS_TURRET_IO = true;
    private static final boolean HAS_SHOOTER_IO = true;
    private static final boolean HAS_HOOD_IO = true;
    private static final boolean HAS_IDX_IO = true;
    private static final boolean HAS_INTAKER_ROLLER_IO = true;
    private static final boolean HAS_INTAKER_EXTENSION_IO = true;
    private static final boolean HAS_SWERVE_IO = true;
    private static final boolean HAS_LL_IO = true;
    private static final boolean HAS_CLIMBER_IO = false;
    private final LimelightSubsystem limelightSubsystem;
    private final IntakerSubsystem intake;
    private final PositionMotorSubsystem<MotorInputsAutoLogged, MotorIO, Distance> climber;
    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);
    private final CommandXboxController tester = new CommandXboxController(2);
    private final ShotCalculator shotCalculator = new ShotCalculator();
    private final Swerve swerve;
    private final TurretSubsystem turret;
    private final VelocityMotorSubsystem<MotorInputsAutoLogged, MotorIO> shooter;
    private final SpindexerSubsystem spindexer;
    private final PositionMotorSubsystem<MotorInputsAutoLogged, MotorIO, Angle> hood;
    private final ShootingSuperstructure shootingSuperstructure;
    private final VelocityMotorSubsystem<MotorInputsAutoLogged, MotorIO> intakerRoller;
    private final PositionMotorSubsystem<MotorInputsAutoLogged, MotorIO, Distance> intakerExtension;
    private final IndicatorSubsystem indicatorSubsystem;
    private final CANCoderIOSim encoderG1Sim = new CANCoderIOSim();
    private final CANCoderIOSim encoderG2Sim = new CANCoderIOSim();

    @SneakyThrows
    public RobotContainer() {

        RobotModeTriggers.teleop().onTrue(Commands.runOnce(HubShiftUtil::initialize));
        RobotModeTriggers.autonomous().onTrue(Commands.runOnce(HubShiftUtil::initialize));
        RobotModeTriggers.disabled()
                .onTrue(Commands.runOnce(HubShiftUtil::initialize).ignoringDisable(true));
        FieldView.Init();
        SignalLogger.enableAutoLogging(false);
        final boolean isReal = RobotBase.isReal();

        swerve = buildSwerve(isReal && HAS_SWERVE_IO);
        limelightSubsystem = buildLimelight(isReal && HAS_LL_IO, swerve);

        turret = buildTurret(isReal && HAS_TURRET_IO);
        shooter = buildShooter(isReal && HAS_SHOOTER_IO);
        spindexer = buildSpindexer(isReal && HAS_IDX_IO);

        climber = buildClimber(isReal && HAS_CLIMBER_IO);

        indicatorSubsystem = buildIndicator(isReal);

        hood = buildHood(isReal && HAS_HOOD_IO);

        intakerRoller = buildIntakerRoller(isReal && HAS_INTAKER_ROLLER_IO);
        intakerExtension = buildIntakerExtension(isReal && HAS_INTAKER_EXTENSION_IO);

        Path deploy = Filesystem.getDeployDirectory().toPath();
        shotCalculator.initialize(
                Map.of(
                        TargetMode.GOAL,
                        deploy.resolve("results_GOAL.json"),
                        TargetMode.FEED,
                        deploy.resolve("results_PASS.json")));

        shootingSuperstructure = new ShootingSuperstructure(turret, hood, shooter, spindexer);
        intake = new IntakerSubsystem(intakerRoller, intakerExtension);
        AutoActions.init(
                swerve,
                shootingSuperstructure,
                shotCalculator,
                intake,
                climber,
                shooter,
                intakerExtension);
        AutoRoutines.init(swerve, shooter, spindexer, intakerExtension);
        AutoFile.init();
        configureBindings();
        shootingSuperstructure.setDefaultCommand();
        swerve.setDefaultCommand(
                SwerveCommands.driveWithJoystick(
                        swerve,
                        () -> -driver.getLeftY(),
                        () -> -driver.getLeftX(),
                        () -> -driver.getRightX(),
                        RobotStateRecorder::getPoseDriverRobotCurrent,
                        MetersPerSecond.of(0.04),
                        DegreesPerSecond.of(3.0)));
        intake.setDefaultCommand();

        climber.setDefaultCommand(climber.runStop());

        indicatorSubsystem.setDefaultCommand(
                Commands.runOnce(
                                () -> {
                                    if (!DriverStation.isEnabled()) {
                                        if (DriverStation.isDSAttached()) {
                                            indicatorSubsystem.setPattern(
                                                    AllianceFlipUtil.shouldFlip()
                                                            ? Patterns.RED_ALLIANCE
                                                            : Patterns.BLUE_ALLIANCE);
                                        } else {
                                            indicatorSubsystem.setPattern(Patterns.LOSS);
                                        }
                                    } else {
                                        if (DriverStation.isAutonomousEnabled()) {
                                            indicatorSubsystem.setPattern(Patterns.AUTO);
                                        } else {
                                            if (shootingSuperstructure.isShooting()) {
                                                indicatorSubsystem.setPattern(Patterns.SHOOTING);
                                            } else {
                                                if (intake.getCurrentMode()
                                                        == frc.robot.subsystems.IntakerSubsystem
                                                                .IntakeMode.INTAKING) {
                                                    indicatorSubsystem.setPattern(Patterns.INTAKE);
                                                } else {
                                                    indicatorSubsystem.setPattern(Patterns.NORMAL);
                                                }
                                            }
                                        }
                                    }
                                },
                                indicatorSubsystem)
                        .onlyIf(() -> !indicatorSubsystem.isOutsideDefault())
                        .ignoringDisable(true));
    }

    public void robotPeriodic() {
        Logger.recordOutput("Simulation/StationaryPose", new Pose3d());

        // Visualize robot components (turret, intake, spindexer)
        shootingSuperstructure.visualizeComponents(
                intakerExtension.getCurrPos().in(Meters), spindexer.getPosition().in(Radians));

        // Visualize projectile balls shot at 0.1s intervals
        shootingSuperstructure.visualizeProjectileBalls(Timer.getTimestamp());

        // update IO inputs
        PhoenixUtils.refreshAll();
        // update NTparameters
        // if (!Robot.isReal()) {
        NTParameterRegistry.refresh();
        // }

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
        RobotStateRecorder.putVelocityRobotCmd(now, swerve.getChassisSpeedsCmd());
        RobotStateRecorder.putOmegaRobotCurrent(
                RadiansPerSecond.of(swerve.getYawVelocityRadPerSec()));
        RobotStateRecorder.setCmdFrame(shotCalculator.computeShotFrame());
        RobotStateRecorder.setCurrentFrame(shootingSuperstructure.getCurrentFrame());
        RobotStateRecorder.periodic();
        FieldView.updateRobotPose(RobotStateRecorder.getPoseWorldRobotCurrent().toPose2d());

        // Visualize robot components (turret, intake, spindexer)
        shootingSuperstructure.visualizeComponents(
                intakerExtension.getCurrPos().in(Meters), spindexer.getPosition().in(Radians));
        // TODO: test & fix in sim
        if (Robot.isReal()) {
            FieldView.updateObjectPose(limelightSubsystem.getPose(LimeLightConfig.NAME_A), "LL_4a");
            FieldView.updateObjectPose(limelightSubsystem.getPose(LimeLightConfig.NAME_B), "LL_4b");
        }

        Logger.recordOutput("Competition/isHubActive", HubShiftUtil.getShiftedShiftInfo().active());
        Logger.recordOutput(
                "Competition/Hub Phase",
                HubShiftUtil.getOfficialShiftInfo().currentShift().toString());
        Logger.recordOutput(
                "Competition/Hub Remaining", HubShiftUtil.getShiftedShiftInfo().remainingTime());
        SmartDashboard.putBoolean(
                "Competition/isHubActive", HubShiftUtil.getShiftedShiftInfo().active());
        SmartDashboard.putString(
                "Competition/Hub Phase",
                HubShiftUtil.getOfficialShiftInfo().currentShift().toString());
        SmartDashboard.putNumber(
                "Competition/Hub Remaining", HubShiftUtil.getShiftedShiftInfo().remainingTime());

        swerve.setSwerveModuleLimit(getShootingLimit());
    }

    private SwerveModuleLimit getShootingLimit() {
        if (shootingSuperstructure.isGoalShooting
                && shotCalculator.decideShotMode() == TargetMode.GOAL
                && !DriverStation.isAutonomous()) {
            // Get current shot parameters to determine distance
            double distance = shootingSuperstructure.getDistance();

            if (distance > 4.8) {
                return SwerveMK5Config.kLongRangeShootingSwerveLimit;
            }

            return SwerveMK5Config.kShootingSwerveLimit;
        }
        return SwerveMK5Config.kDefaultSwerveModuleLimit;
    }

    private void configureBindings() {
        operator.x() // L win
                .onTrue(
                        Commands.runOnce(
                                        () -> {
                                            HubShiftUtil.setAllianceWinOverride(
                                                    () -> Optional.of(true));
                                            Logger.recordOutput("Competition/AutoResultSent", true);
                                            SmartDashboard.putBoolean(
                                                    "Competition/AutoResultSent", true);
                                            Logger.recordOutput("Competition/AutoResultWin", true);
                                            SmartDashboard.putBoolean(
                                                    "Competition/AutoResultWin", true);
                                        })
                                .ignoringDisable(true));
        operator.b() // R lose
                .onTrue(
                        Commands.runOnce(
                                        () -> {
                                            HubShiftUtil.setAllianceWinOverride(
                                                    () -> Optional.of(false));
                                            SmartDashboard.putBoolean(
                                                    "Competition/AutoResultSent", true);
                                            SmartDashboard.putBoolean(
                                                    "Competition/AutoResultSent", true);
                                            Logger.recordOutput("Competition/AutoResultWin", false);
                                            SmartDashboard.putBoolean(
                                                    "Competition/AutoResultWin", false);
                                        })
                                .ignoringDisable(true));

        operator.leftBumper().whileTrue(intake.runExtendedReverse());
        operator.y().onTrue(shootingSuperstructure.runZero());
        operator.a().whileTrue(shootingSuperstructure.runSetFrame());
        operator.leftTrigger().whileTrue(intake.runFeed());
        operator.rightBumper().whileTrue(shootingSuperstructure.runUnjamming());

        driver.leftBumper().onTrue(intake.toggleIntake());
        driver.povUp().onTrue(intake.outZeroCommand());
        driver.povLeft().onTrue(intake.zeroCommand());
        driver.povDown().onTrue(intake.runRetract());
        driver.leftTrigger().whileTrue(intake.runFeed());
        driver.rightTrigger().whileTrue(shootingSuperstructure.runSetFrame());

        // driver.rightTrigger().whileTrue(drivePastNearestSlope());

        operator.rightTrigger()
                .whileTrue(
                        shootingSuperstructure
                                .shootWhenReady(false, driver.getHID(), operator.getHID())
                                .finallyDo(
                                        () -> {
                                            swerve.setSwerveModuleLimitDefault();
                                            CommandScheduler.getInstance()
                                                    .schedule(
                                                            indicatorSubsystem.indicateWithTimeout(
                                                                    Patterns.AFTER_SHOOTING, 0.5));
                                        }));

        driver.rightBumper()
                .whileTrue(
                        shootingSuperstructure
                                .shootWhenReady(false, driver.getHID(), operator.getHID())
                                .finallyDo(
                                        () -> {
                                            swerve.setSwerveModuleLimitDefault();
                                            CommandScheduler.getInstance()
                                                    .schedule(
                                                            indicatorSubsystem.indicateWithTimeout(
                                                                    Patterns.AFTER_SHOOTING, 0.5));
                                        }));
        driver.leftStick()
                .whileTrue(
                        Commands.parallel(
                                SwerveCommands.xLock(swerve),
                                shootingSuperstructure.shootWhenReady(false)));

        operator.rightTrigger()
                .or(driver.rightBumper())
                .or(driver.leftStick())
                .and(shootingSuperstructure::isInTower)
                .whileTrue(indicatorSubsystem.indicate(Patterns.HOLD_SHOOTING));

        operator.rightTrigger()
                .or(driver.rightBumper())
                .or(driver.leftStick())
                .and(shootingSuperstructure::isInHubZone)
                .whileTrue(indicatorSubsystem.indicate(Patterns.HOLD_SHOOTING));

        // SYSID/test
        SysIdCommand shooterSysId = new SysIdCommand(shooter);
        driver.a().whileTrue(shooterSysId.quasistatic(SysIdRoutine.Direction.kForward));
        driver.b().whileTrue(shooterSysId.quasistatic(SysIdRoutine.Direction.kReverse));
        driver.x().whileTrue(shooterSysId.dynamic(SysIdRoutine.Direction.kForward));
        driver.y().whileTrue(shooterSysId.dynamic(SysIdRoutine.Direction.kReverse));

        // SysIdCommand spindexerSysId = new SysIdCommand(spindexer);

        // tester.povDown().whileTrue(spindexerSysId.quasistatic(Direction.kForward));

        // tester.povRight().whileTrue(spindexerSysId.quasistatic(Direction.kReverse));

        // tester.povLeft().whileTrue(spindexerSysId.dynamic(Direction.kForward));
        // tester.povUp().whileTrue(spindexerSysId.dynamic(Direction.kReverse));
        // tester.povDown().whileTrue(spindexer.runVelVolt(() -> RotationsPerSecond.of(2.3)));
        // tester.back().onTrue(turret.setCurrentPosition(Degrees.of(-135)).ignoringDisable(true));
        // tester.povUp().onTrue(turret.setTurretPoseWorld(() -> Degrees.of(0)));
        // tester.povRight().onTrue(turret.setTurretPoseWorld(() -> Degrees.of(90)));
        // tester.povDown().onTrue(turret.setTurretPoseWorld(() -> Degrees.of(180)));
        // tester.povLeft().onTrue(turret.setTurretPoseWorld(() -> Degrees.of(270)));

        // SysIdRoutine swerveSysId =
        //         SwerveCommands.sysid(
        //                 swerve,
        //                 Volts.per(Seconds).of(SysIdCommandParams.rampRateVoltsPerSecond),
        //                 Volts.of(SysIdCommandParams.stepVoltageVolts),
        //                 Seconds.of(SysIdCommandParams.timeoutSeconds));
        // driver.a().whileTrue(swerveSysId.quasistatic(SysIdRoutine.Direction.kForward));
        // driver.b().whileTrue(swerveSysId.quasistatic(SysIdRoutine.Direction.kReverse));
        // driver.x().whileTrue(swerveSysId.dynamic(SysIdRoutine.Direction.kForward));
        // driver.y().whileTrue(swerveSysId.dynamic(SysIdRoutine.Direction.kReverse));
        // driver.back().whileTrue(intakerExtension.zeroCommand());
        // driver.povUp().onTrue(hood.zeroCommand());

        // driver.x()
        //         .whileTrue(
        //                 shooter.runVelVolt(
        //                                 () ->
        //                                         RotationsPerSecond.of(
        //                                                 ShooterParamsNT.testVelRPS.getValue()))
        //                         .alongWith(
        //                                 hood.runPosition(
        //                                         () ->
        //                                                 Degrees.of(
        //
        // HoodParamsNT.testAngle.getValue())))
        //
        // .beforeStarting(Commands.runOnce(serialSubsystem::startMeasurement))
        //                         .finallyDo(serialSubsystem::stopMeasurement));

        // driver.rightTrigger()
        //         .whileTrue(
        //                 spindexer.runVelVolt(
        //
        // RotationsPerSecond.of(SpindexerModeParamsNT.feedRPS.getValue())));

        // driver.a()
        //         .whileTrue(
        //                 new SwerveDriveToPose(
        //                         swerve,
        //                         () -> RobotStateRecorder.getPoseWorldRobotCurrent(),
        //                         () -> AllianceFlipUtil.apply(new
        // Pose3d(AutoActions.kSlopeFrontL)),
        //                         () -> RobotStateRecorder.getVelocityWorldRobotCurrent(),
        //                         new PIDController(
        //                                 AutoParamsNT.kpStrave.getValue(),
        //                                 AutoParamsNT.kiStrave.getValue(),
        //                                 AutoParamsNT.kdStrave.getValue()),
        //                         new PIDController(
        //                                 AutoParamsNT.kpSpin.getValue(),
        //                                 AutoParamsNT.kiSpin.getValue(),
        //                                 AutoParamsNT.kdSpin.getValue()),
        //                         Meters.of(0.2),
        //                         Degrees.of(2)));

        // driver.a().whileTrue(AutoActions.allignToClimb(false));

        new Trigger(DriverStation::isEnabled).onTrue(hood.zeroCommand());

        operator.rightStick()
                .onTrue(shootingSuperstructure.runResetBallCounter().ignoringDisable(true));

        // .alongWith(intakerExtension.zeroCommand()));

        // Swerve
        driver.start()
                .onTrue(
                        Commands.sequence(
                                        SwerveCommands.resetAngle(
                                                        swerve,
                                                        () ->
                                                                AllianceFlipUtil.shouldFlip()
                                                                        ? Rotation2d.k180deg
                                                                        : Rotation2d.kZero)
                                                .alongWith(
                                                        Commands.runOnce(
                                                                () ->
                                                                        RobotStateRecorder
                                                                                .getInstance()
                                                                                .resetTransform(
                                                                                        TransformRecorder
                                                                                                .kFrameWorld,
                                                                                        TransformRecorder
                                                                                                .kFrameRobot))),
                                        Commands.runOnce(
                                                limelightSubsystem::requestInternalIMUReseedAll))
                                .alongWith(
                                        indicatorSubsystem.indicateWithTimeout(
                                                Patterns.RESET_ODOM, 1))
                                .ignoringDisable(true));
    }

    // Helper methods

    private SpindexerSubsystem buildSpindexer(boolean isReal) {
        return new SpindexerSubsystem(
                IdxConfig.SPINDEXER_CFG,
                new MotorInputsAutoLogged(),
                isReal
                        ? new MotorIOTalonFX(IdxConfig.SPINDEXER_CFG)
                        : new MotorIOSim(IdxConfig.SPINDEXER_CFG),
                SpindexerParamsNT.asVelocityParamSources());
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

    private IndicatorSubsystem buildIndicator(boolean isReal) {
        return new IndicatorSubsystem(
                isReal ? new IndicatorIOARGB(LED_PORT, LED_LENGTH) : new IndicatorIOSim());
    }

    public void setThrottle(boolean enabled) {

        limelightSubsystem.setThrottleAll(enabled);
    }

    private LimelightSubsystem buildLimelight(boolean isReal, Swerve swerve) {
        return isReal
                ? new LimelightSubsystem(
                        swerve,
                        new LimelightIOReal(
                                LimeLightConfig.limelightAConfig,
                                () ->
                                        RobotStateRecorder.getPoseWorldRobotCurrent()
                                                .toPose2d()
                                                .getRotation()
                                                .getDegrees(),
                                () ->
                                        RobotStateRecorder.getVelocityWorldRobotCurrent()
                                                .getRotation()
                                                .getDegrees(),
                                () ->
                                        Math.abs(
                                                        RobotStateRecorder
                                                                .getVelocityWorldRobotCurrent()
                                                                .getRotation()
                                                                .getRadians())
                                                > 1.2,
                                LimeLightConfig.asDeviationParams()),
                        new LimelightIOReal(
                                LimeLightConfig.limelightBConfig,
                                () ->
                                        RobotStateRecorder.getPoseWorldRobotCurrent()
                                                .toPose2d()
                                                .getRotation()
                                                .getDegrees(),
                                () ->
                                        RobotStateRecorder.getVelocityWorldRobotCurrent()
                                                .getRotation()
                                                .getDegrees(),
                                () ->
                                        Math.abs(
                                                        RobotStateRecorder
                                                                .getVelocityWorldRobotCurrent()
                                                                .getRotation()
                                                                .getRadians())
                                                > 1.2,
                                LimeLightConfig.asDeviationParams()))
                : new LimelightSubsystem(swerve); // TODO: sim
    }

    private TurretSubsystem buildTurret(boolean isReal) {
        return new TurretSubsystem(
                TurretConfig.TURRET_CONFIG,
                new MotorInputsAutoLogged(),
                isReal
                        ? new MotorIOTalonFX(TurretConfig.TURRET_CONFIG)
                        : new MotorIOSim(TurretConfig.TURRET_CONFIG),
                isReal
                        ? new CANCoderIOCANCoder(
                                TurretConfig.TURRET_ENCODER_G1_ID,
                                ROBORIO_CAN_BUS,
                                TurretConfig.TURRET_ENCODER_G1_OFFSET,
                                false)
                        : encoderG1Sim,
                isReal
                        ? new CANCoderIOCANCoder(
                                TurretConfig.TURRET_ENCODER_G2_ID,
                                ROBORIO_CAN_BUS,
                                TurretConfig.TURRET_ENCODER_G2_OFFSET,
                                false)
                        : encoderG2Sim,
                TurretVelParamsNT.asVelocityParamSources());
    }

    private VelocityMotorSubsystem<MotorInputsAutoLogged, MotorIO> buildShooter(boolean isReal) {
        return new VelocityMotorSubsystem<>(
                ShooterConfig.SHOOTER_CONFIG,
                new MotorInputsAutoLogged(),
                isReal
                        ? new MotorIOTalonFX(ShooterConfig.SHOOTER_CONFIG)
                        : new MotorIOSim(ShooterConfig.SHOOTER_CONFIG),
                ShooterParamsNT.asVelocityParamSources());
    }

    private PositionMotorSubsystem<MotorInputsAutoLogged, MotorIO, Angle> buildHood(
            boolean isReal) {
        return new PositionMotorSubsystem<>(
                HoodConfig.HOOD_CONFIG,
                new MotorInputsAutoLogged(),
                isReal
                        ? new MotorIOTalonFX(HoodConfig.HOOD_CONFIG)
                        : new MotorIOSim(HoodConfig.HOOD_CONFIG),
                HoodParamsNT.asPositionParamSources(),
                Degrees.of(HoodConfig.HOOD_ANGLE_ZEROED_DEG),
                Degrees.of(360.0));
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

    private PositionMotorSubsystem<MotorInputsAutoLogged, MotorIO, Distance> buildIntakerExtension(
            boolean isReal) {
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

    private PositionMotorSubsystem<MotorInputsAutoLogged, MotorIO, Distance> buildClimber(
            boolean isReal) {
        return new PositionMotorSubsystem<>(
                ClimberConfiig.CLIMBER_CONFIG,
                new MotorInputsAutoLogged(),
                isReal
                        ? new MotorIOTalonFX(ClimberConfiig.CLIMBER_CONFIG)
                        : new MotorIOSim(ClimberConfiig.CLIMBER_CONFIG),
                ClimberParamsNT.asPositionParamSources(),
                Meters.of(0),
                ClimberConfiig.CLIMB_METERS_PER_ROTATION);
    }

    public Command zeroIntake() {
        return intake.zeroCommand();
    }

    public Command getAutonomousCommand() {
        return AutoFile.buildAuto();
        // return AutoRoutines.rightLongFuel();
    }
}
