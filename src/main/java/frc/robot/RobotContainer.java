package frc.robot;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.RobotConstants.ROBORIO_CAN_BUS;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.DriveFeedforwards;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Configs.*;
import java.nio.file.Path;
import java.util.Map;
import lib.ironpulse.io.*;
import lib.ironpulse.subsystem.position.PositionMotorSubsystem;
import lib.ironpulse.subsystem.velocity.VelocityMotorSubsystem;
import lib.ironpulse.swerve.*;
import lib.ironpulse.swerve.mk5n.*;
import lib.ironpulse.swerve.sim.*;

public class RobotContainer {

    private static final boolean HAS_SPINDEXER_IO = true;
    private static final boolean HAS_SWERVE_IO = true;
    private static final boolean HAS_SHOOTER_IO = true;
    private static final boolean HAS_TURRET_IO = true;
    private static final boolean HAS_HOOD_IO = true;

    private final CommandXboxController driver = new CommandXboxController(0);

    private final Swerve swerve;
    private final SpindexerSubsystem spindexer;
    private final ShootingSuperstructure shootingSuperstructure;
    private final TurretSubsystem turret;
    private final PositionMotorSubsystem<MotorInputsAutoLogged, MotorIO, Angle> hood;
    private final VelocityMotorSubsystem<MotorInputsAutoLogged, MotorIO> shooter;
    private final ShotCalculator shotCalculator;
    private final SendableChooser<Command> autoChooser;

    private final CANCoderIOSim encoderG1Sim = new CANCoderIOSim();
    private final CANCoderIOSim encoderG2Sim = new CANCoderIOSim();

    public RobotContainer() {

        boolean isReal = RobotBase.isReal();

        swerve = buildSwerve(isReal && HAS_SWERVE_IO);

        spindexer = buildSpindexer(isReal && HAS_SPINDEXER_IO);
        turret = buildTurret(isReal && HAS_TURRET_IO);
        hood = buildHood(isReal && HAS_HOOD_IO);
        shooter = buildShooter(isReal && HAS_SHOOTER_IO);

        shootingSuperstructure = new ShootingSuperstructure(shooter, hood, turret, spindexer);

        shotCalculator = new ShotCalculator();
        shotCalculator.initialize(
                Map.of(
                        ShotCalculator.TargetMode.GOAL,
                        Path.of(
                                Filesystem.getDeployDirectory().getAbsolutePath(),
                                "results_GOAL.json"),
                        ShotCalculator.TargetMode.FEED,
                        Path.of(
                                Filesystem.getDeployDirectory().getAbsolutePath(),
                                "results_PASS.json")));

        shootingSuperstructure.setDefaultCommand();

        swerve.setDefaultCommand(
                SwerveCommands.driveWithJoystick(
                        swerve,
                        () -> -driver.getLeftY(),
                        () -> -driver.getLeftX(),
                        () -> -driver.getRightX(),
                        () -> new Pose3d(),
                        MetersPerSecond.of(0.04),
                        DegreesPerSecond.of(3.0)));

        configureBindings();
        configureAutoBuilder();

        NamedCommands.registerCommand(
                "runShoot",
                shootingSuperstructure
                        .runShoot()
                        .withTimeout(AutoParamsNT.shootTimeoutSec.getValue()));

        autoChooser = AutoBuilder.buildAutoChooser("Auto Chooser");
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void configureAutoBuilder() {
        AutoBuilder.configure(
                () -> swerve.getEstimatedPose().toPose2d(),
                pose -> swerve.resetEstimatedPose(new Pose3d(pose)),
                swerve::getChassisSpeeds,
                (ChassisSpeeds speeds, DriveFeedforwards feedforwards) -> swerve.runTwist(speeds),
                new PPHolonomicDriveController(
                        new PIDConstants(
                                AutoParamsNT.translationKP.getValue(),
                                AutoParamsNT.translationKI.getValue(),
                                AutoParamsNT.translationKD.getValue()),
                        new PIDConstants(
                                AutoParamsNT.rotationKP.getValue(),
                                AutoParamsNT.rotationKI.getValue(),
                                AutoParamsNT.rotationKD.getValue())),
                RobotConstants.AUTO_ROBOT_CONFIG,
                () -> {
                    var alliance = DriverStation.getAlliance();
                    return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
                },
                swerve);
    }

    private void configureBindings() {

        driver.rightTrigger().whileTrue(shootingSuperstructure.runShoot());
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

    private SpindexerSubsystem buildSpindexer(boolean isReal) {
        return new SpindexerSubsystem(
                SpindexConfig.SPINDEXER_CONFIG,
                new MotorInputsAutoLogged(),
                isReal
                        ? new MotorIOTalonFX(SpindexConfig.SPINDEXER_CONFIG)
                        : new MotorIOSim(SpindexConfig.SPINDEXER_CONFIG),
                SpindexerParamsNT.asVelocityParamSources());
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

    public void robotPeriodic() {
        lib.ntext.NTParameterRegistry.refresh();
        RobotStateRecorder.putPoseWorldRobot(
                swerve.getEstimatedPose(), Seconds.of(Timer.getTimestamp()));

        RobotStateRecorder.setCmdFrame(
                driver.rightTrigger().getAsBoolean()
                        ? shotCalculator.computeGoalFrame()
                        : shotCalculator.computeShotFrame());
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
