package frc.robot.auto;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.robot.RobotStateRecorder;
import frc.robot.subsystems.IntakerSubsystem;
import frc.robot.subsystems.ShootingSubsystem.ShootingSuperstructure;
import frc.robot.subsystems.ShootingSubsystem.ShotCalculator;
import lib.ironpulse.math.rbd.TransformRecorder;
import lib.ironpulse.swerve.Swerve;
import lib.ironpulse.swerve.SwerveCommands;
import lib.ironpulse.swerve.SwerveLimit;
import lib.ironpulse.utils.AllianceFlipUtil;
import org.littletonrobotics.junction.Logger;

public class AutoActions {
    public static final Pose2d kSweepStartPoseL = new Pose2d(7.972, 6.928, new Rotation2d(-90));
    public static final Pose2d kSweepStartPoseR = new Pose2d(7.972, 1.126, new Rotation2d(90));

    private static Swerve swerve;
    private static ShootingSuperstructure shooter;
    private static ShotCalculator shotCalculator;
    private static IntakerSubsystem intake;

    public static void init(
            Swerve swerve,
            ShootingSuperstructure shooterSS,
            ShotCalculator shotCalculator,
            IntakerSubsystem intake) {
        AutoActions.swerve = swerve;
        AutoActions.shooter = shooter;
        AutoActions.shotCalculator = shotCalculator;
        AutoActions.intake = intake;
    }

    public static Command driveToSweepStart() {
        return Commands.none();
    }

    public static Command followPath(PathPlannerPath path) {
        return new FollowPathCommand(
                        path,
                        () -> RobotStateRecorder.getPoseWorldRobotCurrent().toPose2d(),
                        swerve::getChassisSpeeds,
                        (vel, ff) -> {
                            swerve.runTwist(vel);
                        },
                        new PPHolonomicDriveController(
                                new PIDConstants(5.5, 0.0, 0.0),
                                new PIDConstants(3.0, 0.0, 0.1),
                                RobotConstants.LOOPER_DT),
                        RobotConstants.AUTO_ROBOT_CONFIG,
                        () -> false, // do not flip in command, flip done by user before passing
                        swerve)
                .beforeStarting(
                        () ->
                                Logger.recordOutput(
                                        "Temp/Traj", path.getPathPoses().toArray(new Pose2d[0])));
    }

    // Helpermethod
    public static Command resetOnPose(Pose2d pose) {
        var resetPose = new Pose3d(AllianceFlipUtil.apply(pose));

        return SwerveCommands.reset(swerve, resetPose)
                .alongWith(
                        Commands.runOnce(
                                () -> {
                                    RobotStateRecorder.getInstance()
                                            .resetTransform(
                                                    TransformRecorder.kFrameWorld,
                                                    TransformRecorder.kFrameRobot);
                                }))
                .onlyIf(Robot::isSimulation)
                .ignoringDisable(true);
    }

    public static Command limitSwerve(
            double maxVelocityMps,
            double maxAccelerationMps2,
            double maxAngularVelDegps,
            double maxAngularAccelerationDegps2) {
        return Commands.runOnce(
                () ->
                        swerve.setSwerveLimit(
                                SwerveLimit.builder()
                                        .maxLinearVelocity(MetersPerSecond.of(maxVelocityMps))
                                        .maxSkidAcceleration(
                                                MetersPerSecondPerSecond.of(maxAccelerationMps2))
                                        .maxAngularVelocity(DegreesPerSecond.of(maxAngularVelDegps))
                                        .maxAngularAcceleration(
                                                DegreesPerSecondPerSecond.of(
                                                        maxAngularAccelerationDegps2))
                                        .build()));
    }

    public static Command unlimitSwerve() {
        return Commands.runOnce(swerve::setSwerveLimitDefault);
    }
}
