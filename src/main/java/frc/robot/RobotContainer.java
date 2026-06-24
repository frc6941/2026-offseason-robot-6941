package frc.robot;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Configs.*;
import frc.robot.subsystems.SpindexerSubsystem;
import lib.ironpulse.io.*;
import lib.ironpulse.swerve.*;
import lib.ironpulse.swerve.mk5n.*;
import lib.ironpulse.swerve.sim.*;

public class RobotContainer {

    private static final boolean HAS_SPINDEXER_IO = true;
    private static final boolean HAS_SWERVE_IO = true;
    private static final boolean HAS_SHOOTER_IO = true;

    private final CommandXboxController driver = new CommandXboxController(0);

    private final Swerve swerve;
    private final SpindexerSubsystem spindexer;

    public RobotContainer() {

        boolean isReal = RobotBase.isReal();

        swerve = buildSwerve(isReal && HAS_SWERVE_IO);
        spindexer = buildSpindexer(isReal && HAS_SPINDEXER_IO);

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

    private void configureBindings() {}

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

    public void robotPeriodic() {
        lib.ntext.NTParameterRegistry.refresh();
    }

    public Command getAutonomousCommand() {
        return null;
    }
}
