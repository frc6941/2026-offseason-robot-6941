package frc.robot;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Configs.IntakeConfig;
import frc.robot.subsystems.Configs.IntakerRollerParamsNT;
import frc.robot.subsystems.Configs.SwerveMK5Config;
import frc.robot.subsystems.IntakerSubsystem;
import lib.ironpulse.io.*;
import lib.ironpulse.subsystem.velocity.VelocityMotorSubsystem;
import lib.ironpulse.swerve.*;
import lib.ironpulse.swerve.mk5n.*;
import lib.ironpulse.swerve.sim.*;

public class RobotContainer {

    private static final boolean HAS_INTAKER_IO = true;
    private static final boolean HAS_SWERVE_IO = true;

    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);

    private final Swerve swerve;
    private final IntakerSubsystem intake;

    public RobotContainer() {

        boolean isReal = RobotBase.isReal();

        // ===== Swerve =====
        swerve = buildSwerve(isReal && HAS_SWERVE_IO);

        // ===== Intake motors =====
        var roller = buildIntakerRoller(isReal && HAS_INTAKER_IO);
        var topRoller = buildIntakerTopRoller(isReal && HAS_INTAKER_IO);

        // ===== Intake subsystem (3 motor version) =====
        intake = new IntakerSubsystem(roller, topRoller);

        // ===== Default drive =====
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
    }

    private void configureBindings() {

        // RT = intake（核心）
        driver.leftTrigger().whileTrue(intake.runIntake());

        // LT = outtake
        driver.rightTrigger().whileTrue(intake.runOuttake());

        // LB = clear jam
        driver.leftBumper().whileTrue(intake.runOuttake());

        // operator backup control
        operator.rightTrigger().whileTrue(intake.runIntake());
    }

    // ================= Swerve =================

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

    // ================= Intake Motor 1 =================

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

    // ================= Intake Motor 2 (NEW 33) =================

    private VelocityMotorSubsystem<MotorInputsAutoLogged, MotorIO> buildIntakerTopRoller(
            boolean isReal) {

        return new VelocityMotorSubsystem<>(
                IntakeConfig.INTAKER_TOP_ROLLER_CONFIG,
                new MotorInputsAutoLogged(),
                isReal
                        ? new MotorIOTalonFX(IntakeConfig.INTAKER_TOP_ROLLER_CONFIG)
                        : new MotorIOSim(IntakeConfig.INTAKER_TOP_ROLLER_CONFIG),
                IntakerRollerParamsNT.asVelocityParamSources());
    }

    // ================= Robot periodic =================

    public void robotPeriodic() {
        lib.ntext.NTParameterRegistry.refresh();
    }

    public Command getAutonomousCommand() {
        return null;
    }
}
