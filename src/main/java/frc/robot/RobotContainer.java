package frc.robot;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Configs.IntakeConfig;
import frc.robot.subsystems.Configs.IntakerExtensionParamsNT;
import frc.robot.subsystems.Configs.IntakerRollerParamsNT;
import frc.robot.subsystems.Configs.SwerveMK5Config;
import frc.robot.subsystems.IntakerSubsystem;
import lib.ironpulse.io.*;
import lib.ironpulse.subsystem.position.PositionMotorSubsystem;
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

        swerve = buildSwerve(isReal && HAS_SWERVE_IO);

        var roller = buildIntakerRoller(isReal && HAS_INTAKER_IO);
        var downRoller = buildIntakerDownRoller(isReal && HAS_INTAKER_IO);
        var extension = buildIntakerExtension(isReal && HAS_INTAKER_IO);

        intake = new IntakerSubsystem(roller, downRoller, extension);
        intake.setDefaultCommand();

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

        // ========== 保留原有 Roller 控制（功能不丢） ==========
        driver.leftTrigger().whileTrue(intake.toggleIntake()).onFalse(intake.runRetract());
        driver.rightTrigger().whileTrue(intake.runIntakeRPS()).onFalse(intake.rampStopRPS());
        driver.leftBumper().whileTrue(intake.runOuttake());

        operator.rightTrigger().whileTrue(intake.runIntakeRPS()).onFalse(intake.rampStopRPS());

        driver.povUp().onTrue(intake.outZeroCommand()); // 外零位
        driver.povLeft().onTrue(intake.zeroCommand()); // 零位

        operator.leftBumper().whileTrue(intake.runExtendedReverse());
        operator.leftTrigger().whileTrue(intake.runFeed());
        operator.povUp().whileTrue(intake.runRetractedFeeding());
        operator.y().onTrue(intake.runExtendedIdle());
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

    private VelocityMotorSubsystem<MotorInputsAutoLogged, MotorIO> buildIntakerDownRoller(
            boolean isReal) {

        return new VelocityMotorSubsystem<>(
                IntakeConfig.INTAKER_DOWN_ROLLER_CONFIG,
                new MotorInputsAutoLogged(),
                isReal
                        ? new MotorIOTalonFX(IntakeConfig.INTAKER_DOWN_ROLLER_CONFIG)
                        : new MotorIOSim(IntakeConfig.INTAKER_DOWN_ROLLER_CONFIG),
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
                Meters.of(IntakeConfig.INTAKE_EXTENSION_METERS_PER_ROTATION));
    }

    public void robotPeriodic() {
        lib.ntext.NTParameterRegistry.refresh();
    }

    public Command getAutonomousCommand() {
        return null;
    }
}
