// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import static frc.robot.RobotConstants.LED_LENGTH;
import static frc.robot.RobotConstants.LED_PORT;
import static frc.robot.RobotConstants.ROBORIO_CAN_BUS;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Configs.*;
import frc.robot.subsystems.IntakerSubsystem;
import frc.robot.subsystems.SerialSubsystem.SerialSubsystem;
import frc.robot.subsystems.ShootingSubsystem.ShootingSuperstructure;
import frc.robot.subsystems.ShootingSubsystem.ShotCalculator;
import frc.robot.subsystems.ShootingSubsystem.ShotCalculator.TargetMode;
import frc.robot.subsystems.ShootingSubsystem.TurretSubsystem;
import java.util.Map;
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
import lib.ironpulse.swerve.mk5n.ImuIOPigeon;
import lib.ironpulse.swerve.mk5n.SwerveModuleIOMK5N;
import lib.ironpulse.swerve.sim.ImuIOSim;
import lib.ironpulse.swerve.sim.SwerveModuleIOSimpleSim;
import lib.ironpulse.utils.AllianceFlipUtil;
import lib.ironpulse.utils.PhoenixUtils;
import lib.ntext.NTParameterRegistry;
import lombok.SneakyThrows;

@SuppressWarnings({"unused"})
public class RobotContainer {
    private static final boolean HAS_TURRET_IO = true;
    private static final boolean HAS_SHOOTER_IO = true;
    private static final boolean HAS_HOOD_IO = true;
    private static final boolean HAS_IDX_IO = true;
    private static final boolean HAS_INTAKER_ROLLER_IO = true;
    private static final boolean HAS_INTAKER_EXTENSION_IO = true;
    private static final boolean HAS_SWERVE_IO = true;
    private static final boolean HAS_LL_IO = true;
    private final LimelightSubsystem limelightSubsystem;
    private final IntakerSubsystem intake;
    private final CommandXboxController driver = new CommandXboxController(0);
    private final ShotCalculator shotCalculator = new ShotCalculator();
    private final Swerve swerve;
    private final TurretSubsystem turret;
    private final VelocityMotorSubsystem<MotorInputsAutoLogged, MotorIO> shooter;
    private final VelocityMotorSubsystem<MotorInputsAutoLogged, MotorIO> spindexer;
    private final PositionMotorSubsystem<MotorInputsAutoLogged, MotorIO, Angle> hood;
    private final ShootingSuperstructure shootingSuperstructure;
    private final VelocityMotorSubsystem intakerRoller;
    private final PositionMotorSubsystem intakerExtension;
    // private final IndicatorSubsystem indicatorSubsystem;
    private final CANCoderIOSim encoderG1Sim = new CANCoderIOSim();
    private final CANCoderIOSim encoderG2Sim = new CANCoderIOSim();

    @SneakyThrows
    public RobotContainer() {
        final boolean isReal = RobotBase.isReal();

        swerve = buildSwerve(isReal && HAS_SWERVE_IO);
        limelightSubsystem = buildLimelight(isReal && HAS_LL_IO, swerve);

        turret = buildTurret(isReal && HAS_TURRET_IO);
        shooter = buildShooter(isReal && HAS_SHOOTER_IO);
        spindexer = buildSpindexer(isReal && HAS_IDX_IO);

        //    indicator = builIndicator(isReal);

        hood = buildHood(isReal && HAS_HOOD_IO);

        intakerRoller = buildIntakerRoller(isReal && HAS_INTAKER_ROLLER_IO);
        intakerExtension = buildIntakerExtension(isReal && HAS_INTAKER_EXTENSION_IO);

        shotCalculator.initialize(
                Map.of(
                        ShotCalculator.TargetMode.GOAL,
                        Filesystem.getDeployDirectory().toPath().resolve("results_GOAL.json")));
        shootingSuperstructure = new ShootingSuperstructure(turret, hood, shooter, spindexer);
        intake = new IntakerSubsystem(intakerRoller, intakerExtension);

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

        // indicatorSubsystem.setDefaultCommand(
        //        indicatorSubsystem.indicate(IndicatorIO.Patterns.NORMAL));
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
        RobotStateRecorder.setCurrentFrame(shootingSuperstructure.getCurrentFrame());
        RobotStateRecorder.periodic();
    }

    private void configureBindings() {
        // INTAKE
        driver.leftTrigger().toggleOnTrue(intake.runIntake());
        driver.a().whileTrue(intake.runFeed());
        driver.povDown().onTrue(intake.runRetract());
        driver.back().whileTrue(intakerExtension.zeroCommand());

        driver.leftBumper()
                .whileTrue(
                        shootingSuperstructure.runFrame(
                                () -> shotCalculator.computeShotFrame(TargetMode.GOAL),
                                () ->
                                        driver.rightTrigger().getAsBoolean()
                                                ? ShootingSuperstructure.IdxMode.FEED
                                                : ShootingSuperstructure.IdxMode.OFF));

        // driver.povLeft().whileTrue(intakerExtension.runPosition(Centimeters.of(32)));
        // driver.povRight().whileTrue(intakerExtension.runPosition(Centimeters.of(4)));
        // driver.leftTrigger().whileTrue(intakerRoller.runDutyCycle(1));

        // SYSID/test
        // driver.leftBumper()
        // .whileTrue(
        //         shootingSuperstructure.runFrame(
        //                 () ->
        //                         new ShotFrame(
        //                                 shotCalculator
        //                                         .getShotToTargetTranslation(TargetMode.GOAL)
        //                                         .getAngle()
        //                                         .getMeasure(),
        //                                 Degrees.of(45),
        //                                 MetersPerSecond.of(12)),
        //                 () ->
        //                         driver.rightTrigger().getAsBoolean()
        //                                 ? ShootingSuperstructure.IdxMode.FEED
        //                                 : ShootingSuperstructure.IdxMode.OFF));
        // driver.rightBumper()
        //         .whileTrue(
        //                 shootingSuperstructure.runFrame(
        //                         () ->
        //                                 new ShotFrame(
        //                                         Degrees.of(43),
        //                                         Degrees.of(45),
        //                                         MetersPerSecond.of(12)),
        //                         () ->
        //                                 driver.rightTrigger().getAsBoolean()
        //                                         ? ShootingSuperstructure.IdxMode.FEED
        //                                         : ShootingSuperstructure.IdxMode.OFF));
        // SysIdCommand shooterSysId = new SysIdCommand(shooter);
        // driver.a().whileTrue(shooterSysId.quasistatic(SysIdRoutine.Direction.kForward));
        // driver.b().whileTrue(shooterSysId.quasistatic(SysIdRoutine.Direction.kReverse));
        // driver.x().whileTrue(shooterSysId.dynamic(SysIdRoutine.Direction.kForward));
        // driver.y().whileTrue(shooterSysId.dynamic(SysIdRoutine.Direction.kReverse));

        // SysIdCommand spindexerSysId = new SysIdCommand(spindexer);
        // driver.povDown().whileTrue(spindexerSysId.quasistatic(SysIdRoutine.Direction.kForward));
        // driver.povRight().whileTrue(spindexerSysId.quasistatic(SysIdRoutine.Direction.kReverse));
        // driver.povLeft().whileTrue(spindexerSysId.dynamic(SysIdRoutine.Direction.kForward));
        // driver.povUp().whileTrue(spindexerSysId.dynamic(SysIdRoutine.Direction.kReverse));
        // driver.povDown().whileTrue(spindexer.runVelVolt(() -> RotationsPerSecond.of(2.3)));
        // driver.back().onTrue(turret.setCurrentPosition(Degrees.of(-135)).ignoringDisable(true));
        // driver.povUp().onTrue(turret.setTurretPoseWorld(() -> Degrees.of(0)));
        // driver.povRight().onTrue(turret.setTurretPoseWorld(() -> Degrees.of(90)));
        // driver.povDown().onTrue(turret.setTurretPoseWorld(() -> Degrees.of(180)));
        // driver.povLeft().onTrue(turret.setTurretPoseWorld(() -> Degrees.of(270)));

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

        // Swerve
        driver.start()
                .onTrue(
                        SwerveCommands.resetAngle(
                                        swerve,
                                        () ->
                                                AllianceFlipUtil.shouldFlip()
                                                        ? Rotation2d.k180deg
                                                        : Rotation2d.kZero)
                                .alongWith(
                                        Commands.runOnce(
                                                () -> {
                                                    RobotStateRecorder.getInstance()
                                                            .resetTransform(
                                                                    TransformRecorder.kFrameWorld,
                                                                    TransformRecorder.kFrameRobot);
                                                }))
                                .ignoringDisable(true));

        new Trigger(DriverStation::isEnabled)
                .onTrue(
                        new InstantCommand(() -> limelightSubsystem.setThrottleAll(true))
                                .alongWith(hood.zeroCommand()));

        new Trigger(DriverStation::isDisabled)
                .onTrue(
                        new InstantCommand(() -> limelightSubsystem.setThrottleAll(false))
                                .ignoringDisable(true));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }

    private VelocityMotorSubsystem<MotorInputsAutoLogged, MotorIO> buildSpindexer(boolean isReal) {
        return new VelocityMotorSubsystem<>(
                IdxConfig.SPINDEXER_CFG,
                new MotorInputsAutoLogged(),
                isReal
                        ? new MotorIOTalonFX(IdxConfig.SPINDEXER_CFG)
                        : new MotorIOSim(IdxConfig.SPINDEXER_CFG),
                SpindexerParamsNT.asVelocityParamSources());
    }

    private Swerve buildSwerve(boolean isReal) {
        final boolean useRealSwerve = isReal && HAS_SWERVE_IO;

        return new Swerve(
                useRealSwerve ? SwerveMK5Config.kRealConfig : SwerveMK5Config.kSimConfig,
                useRealSwerve ? new ImuIOPigeon(SwerveMK5Config.kRealConfig) : new ImuIOSim(),
                useRealSwerve
                        ? new SwerveModuleIOMK5N(SwerveMK5Config.kRealConfig, 0)
                        : new SwerveModuleIOSimpleSim(SwerveMK5Config.kSimConfig, 0),
                useRealSwerve
                        ? new SwerveModuleIOMK5N(SwerveMK5Config.kRealConfig, 1)
                        : new SwerveModuleIOSimpleSim(SwerveMK5Config.kSimConfig, 1),
                useRealSwerve
                        ? new SwerveModuleIOMK5N(SwerveMK5Config.kRealConfig, 2)
                        : new SwerveModuleIOSimpleSim(SwerveMK5Config.kSimConfig, 2),
                useRealSwerve
                        ? new SwerveModuleIOMK5N(SwerveMK5Config.kRealConfig, 3)
                        : new SwerveModuleIOSimpleSim(SwerveMK5Config.kSimConfig, 3));
    }

    private IndicatorSubsystem buildIndicator(boolean isReal) {
        return new IndicatorSubsystem(
                isReal ? new IndicatorIOARGB(LED_PORT, LED_LENGTH) : new IndicatorIOSim());
    }

    private LimelightSubsystem buildLimelight(boolean isReal, Swerve swerve) {
        return isReal
                ? new LimelightSubsystem(
                        swerve,
                        new LimelightIOReal(
                                LimeLightConfig.limelight1Config,
                                () ->
                                        RobotStateRecorder.getPoseWorldRobotCurrent()
                                                .toPose2d()
                                                .getRotation()
                                                .getDegrees(),
                                () ->
                                        RobotStateRecorder.getVelocityWorldRobotCurrent()
                                                        .getRotation()
                                                        .getDegrees()
                                                > 360,
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

    private SerialSubsystem buildSerial(boolean isReal) {
        return new SerialSubsystem();
    }
}
