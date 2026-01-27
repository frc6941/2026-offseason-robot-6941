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
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.Configs.HoodConfig;
import frc.robot.subsystems.Configs.HoodParamsNT;
import frc.robot.subsystems.Configs.IntakerConfig;
import frc.robot.subsystems.Configs.IntakerParamsNT;
import frc.robot.subsystems.Configs.ShooterConfig;
import frc.robot.subsystems.Configs.ShooterParamsNT;
import frc.robot.subsystems.Configs.IdxConfig;
import frc.robot.subsystems.Configs.IdxHorizParamsNT;
import frc.robot.subsystems.Configs.IdxSpinParamsNT;
import frc.robot.subsystems.Configs.IdxVertParamsNT;
import frc.robot.subsystems.Configs.SwerveMK5Config;
import frc.robot.subsystems.Configs.TurretConfig;
import frc.robot.subsystems.Configs.TurretVelParamsNT;
import frc.robot.subsystems.ShootingSubsystem.ShotCalculator;
import frc.robot.subsystems.ShootingSubsystem.ShootingSuperstructure;
import frc.robot.subsystems.ShootingSubsystem.SpindexerSubsystem;
import frc.robot.subsystems.ShootingSubsystem.TurretSubsystem;
import frc.robot.subsystems.ShootingSubsystem.TurretSubsystem.TurretMode;
import lib.ironpulse.command.SysIdCommand;
import lib.ironpulse.command.VisualizeProjectileShot;
import lib.ironpulse.io.CANCoderIOCANCoder;
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
import lib.ironpulse.swerve.mk5n.ImuIOPigeon;
import lib.ironpulse.swerve.mk5n.SwerveModuleIOMK5N;
import lib.ironpulse.swerve.sim.ImuIOSim;
import lib.ironpulse.swerve.sim.SwerveModuleIOSimpleSim;
import lib.ironpulse.utils.PhoenixUtils;
import lib.ntext.NTParameterRegistry;

@SuppressWarnings({"unused"})
public class RobotContainer {
    private static final boolean HAS_TURRET_IO = false;
    private static final boolean HAS_SHOOTER_IO = false;
    private static final boolean HAS_HOOD_IO = false;
    private static final boolean HAS_IDX_IO = true;

    private final CommandXboxController driver = new CommandXboxController(0);
    private final ShotCalculator shotCalculator = new ShotCalculator();
    private final Swerve swerve;
    private final TurretSubsystem turret;
    private final VelocityMotorSubsystem<MotorInputsAutoLogged, MotorIO> shooter;
    private final SpindexerSubsystem idx;
    private final PositionMotorSubsystem<MotorInputsAutoLogged, MotorIO, Angle> hood;
    //private final ShootingSuperstructure shootingSuperstructure;
    private final CANCoderIOSim encoderG1Sim = new CANCoderIOSim();
    private final CANCoderIOSim encoderG2Sim = new CANCoderIOSim();

    public RobotContainer() {
        VelocityMotorSubsystem<MotorInputsAutoLogged, MotorIO> spin;
        VelocityMotorSubsystem<MotorInputsAutoLogged, MotorIO> vert;
        VelocityMotorSubsystem<MotorInputsAutoLogged, MotorIO> horiz;

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
                            HAS_TURRET_IO
                                    ? new MotorIOTalonFX(TurretConfig.TURRET_CONFIG)
                                    : new MotorIOSim(TurretConfig.TURRET_CONFIG),
                            HAS_TURRET_IO
                                    ? new CANCoderIOCANCoder(
                                            TurretConfig.TURRET_ENCODER_G1_ID,
                                            TurretConfig.TURRET_ENCODER_G1_OFFSET,
                                            false)
                                    : encoderG1Sim,
                            HAS_TURRET_IO
                                    ? new CANCoderIOCANCoder(
                                            TurretConfig.TURRET_ENCODER_G2_ID,
                                            TurretConfig.TURRET_ENCODER_G2_OFFSET,
                                            false)
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
            spin =
                    new VelocityMotorSubsystem<>(
                            IdxConfig.SPIN_CFG,
                            new MotorInputsAutoLogged(),
                            HAS_IDX_IO
                                    ? new MotorIOTalonFX(IdxConfig.SPIN_CFG)
                                    : new MotorIOSim(IdxConfig.SPIN_CFG),
                            IdxSpinParamsNT.asVelocityParamSources());
            vert =
                    new VelocityMotorSubsystem<>(
                            IdxConfig.VERT_CFG,
                            new MotorInputsAutoLogged(),
                            HAS_IDX_IO
                                    ? new MotorIOTalonFX(IdxConfig.VERT_CFG)
                                    : new MotorIOSim(IdxConfig.VERT_CFG),
                            IdxVertParamsNT.asVelocityParamSources());
            horiz =
                    new VelocityMotorSubsystem<>(
                            IdxConfig.HORIZ_CFG,
                            new MotorInputsAutoLogged(),
                            HAS_IDX_IO
                                    ? new MotorIOTalonFX(IdxConfig.HORIZ_CFG)
                                    : new MotorIOSim(IdxConfig.HORIZ_CFG),
                            IdxHorizParamsNT.asVelocityParamSources());
            hood =
                    new PositionMotorSubsystem<>(
                            HoodConfig.HOOD_CONFIG,
                            new MotorInputsAutoLogged(),
                            HAS_HOOD_IO
                                    ? new MotorIOTalonFX(HoodConfig.HOOD_CONFIG)
                                    : new MotorIOSim(HoodConfig.HOOD_CONFIG),
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
                    new VelocityMotorSubsystem<>(
                            ShooterConfig.SHOOTER_CONFIG,
                            new MotorInputsAutoLogged(),
                            new MotorIOSim(ShooterConfig.SHOOTER_CONFIG),
                            ShooterParamsNT.asVelocityParamSources());
            spin =
                    new VelocityMotorSubsystem<>(
                            IdxConfig.SPIN_CFG,
                            new MotorInputsAutoLogged(),
                            new MotorIOSim(IdxConfig.SPIN_CFG),
                            IdxSpinParamsNT.asVelocityParamSources());
            vert =
                    new VelocityMotorSubsystem<>(
                            IdxConfig.VERT_CFG,
                            new MotorInputsAutoLogged(),
                            new MotorIOSim(IdxConfig.VERT_CFG),
                            IdxVertParamsNT.asVelocityParamSources());
            horiz =
                    new VelocityMotorSubsystem<>(
                            IdxConfig.HORIZ_CFG,
                            new MotorInputsAutoLogged(),
                            new MotorIOSim(IdxConfig.HORIZ_CFG),
                            IdxHorizParamsNT.asVelocityParamSources());
            hood =
                    new PositionMotorSubsystem<>(
                            HoodConfig.HOOD_CONFIG,
                            new MotorInputsAutoLogged(),
                            new MotorIOSim(HoodConfig.HOOD_CONFIG),
                            HoodParamsNT.asPositionParamSources(),
                            Degrees.of(0),
                            Degrees.of(360));
        }

        idx = new SpindexerSubsystem(spin, vert, horiz);
        //shootingSuperstructure =
        //        new ShootingSuperstructure(turret, hood, shooter, idx, shotCalculator);
        configureBindings();
        //shootingSuperstructure.setDefaultCommand();
        idx.setDefaultCommand();
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
        shotCalculator.refreshTuningFromNetworkTables();
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


    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
