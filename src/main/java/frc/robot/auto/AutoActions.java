package frc.robot.auto;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.RotationTarget;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants;
import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.robot.RobotStateRecorder;
import frc.robot.subsystems.Configs.AutoParamsNT;
import frc.robot.subsystems.IntakerSubsystem;
import frc.robot.subsystems.ShootingSubsystem.ShootingSuperstructure;
import frc.robot.subsystems.ShootingSubsystem.ShotCalculator;
import java.util.Collections;
import java.util.List;
import lib.ironpulse.math.rbd.TransformRecorder;
import lib.ironpulse.swerve.Swerve;
import lib.ironpulse.swerve.SwerveCommands;
import lib.ironpulse.swerve.SwerveLimit;
import lib.ironpulse.swerve.commands.SwerveDriveToPose;
import lib.ironpulse.utils.AllianceFlipUtil;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class AutoActions {
    public static final Pose2d kSweepStartPoseL =
            new Pose2d(7.972, 6.928, new Rotation2d(Degrees.of(0)));
    public static final Pose2d kSweepStartPoseR =
            new Pose2d(7.972, 1.126, new Rotation2d(Degrees.of(90)));
    public static final Pose2d kSweepEndPoseL =
            new Pose2d(7.972, 6.928, new Rotation2d(Degrees.of(0)));
    public static final Pose2d kSweepEndPoseR =
            new Pose2d(7.972, 1.126, new Rotation2d(Degrees.of(0)));
    public static final Pose2d kSlopeFrontL =
            new Pose2d(5.84, 5.79, new Rotation2d(Degrees.of(45)));
    public static final Pose2d kSlopeFrontR =
            new Pose2d(5.84, 2.227, new Rotation2d(Degrees.of(45)));


            public static final Pose2d kTestA =
            new Pose2d(5.84, 2.227, new Rotation2d(Degrees.of(45)));
            public static final Pose2d kTestB =
            new Pose2d(5.84, 2.227, new Rotation2d(Degrees.of(45)));
            public static final Pose2d kTestC =
            new Pose2d(5.84, 2.227, new Rotation2d(Degrees.of(45)));          

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
        AutoActions.shooter = shooterSS;
        AutoActions.shotCalculator = shotCalculator;
        AutoActions.intake = intake;
    }

    public static Command driveToSweepStart(boolean isLeft) {
        Pose2d slopeFront = AllianceFlipUtil.apply(isLeft ? kSlopeFrontL : kSlopeFrontR);
        Pose2d sweepStart = AllianceFlipUtil.apply(isLeft ? kSweepStartPoseL : kSweepStartPoseR);

        Command driveToSlopeFront =
                new SwerveDriveToPose(
                        swerve,
                        () -> RobotStateRecorder.getPoseWorldRobotCurrent(),
                        () -> new Pose3d(slopeFront),
                        () -> RobotStateRecorder.getVelocityWorldRobotCurrent(),
                        new PIDController(
                                AutoParamsNT.kpStrave.getValue(),
                                AutoParamsNT.kiStrave.getValue(),
                                AutoParamsNT.kdStrave.getValue()),
                        new PIDController(
                                AutoParamsNT.kpSpin.getValue(),
                                AutoParamsNT.kiSpin.getValue(),
                                AutoParamsNT.kdSpin.getValue()),
                        edu.wpi.first.units.Units.Meters.of(0.2),
                        Degrees.of(2));

        Command waitPitchSettled =
                Commands.waitUntil(() -> isPitchStable() && hasCrossedBump(true));

        Command driveToSweepStart =
                swerve.defer(
                        () -> {
                            Pose2d current =
                                    RobotStateRecorder.getPoseWorldRobotCurrent().toPose2d();
                            List<Pose2d> waypoints = List.of(current, sweepStart);
                            PathPlannerPath path =
                                    generatePath(
                                            waypoints, Collections.emptyList(), 4.2, 10.0, 0.0);
                            return followPath(path);
                        });

        return Commands.sequence(
                Commands.deadline(waitPitchSettled, driveToSlopeFront), driveToSweepStart).alongWith(Commands.runOnce(() -> {
                    Logger.recordOutput("Temp/hasCrossedBump", hasCrossedBump(true));
                    Logger.recordOutput("Temp/pitchStable", isPitchStable());
                }));
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

    public static PathPlannerPath generatePath(
            List<Pose2d> waypoints,
            List<RotationTarget> rotationTargets,
            double maxVel,
            double maxAcc,
            double endVelMps) {
        PathConstraints constraints = new PathConstraints(maxVel, maxAcc, 15.0, 40.0, 12.0);
        List<Waypoint> pts = PathPlannerPath.waypointsFromPoses(waypoints);
        Pose2d lastPose = waypoints.get(waypoints.size() - 1);
        GoalEndState endState = new GoalEndState(endVelMps, lastPose.getRotation());
        return new PathPlannerPath(
                pts,
                rotationTargets,
                Collections.emptyList(),
                Collections.emptyList(),
                Collections.emptyList(),
                constraints,
                null,
                endState,
                false);
    }

    public static PathPlannerPath generatePath(
            List<Pose2d> waypoints, List<RotationTarget> rotationTargets, double endVelMps) {
        return generatePath(waypoints, rotationTargets, 4.5, 7.0, endVelMps);
    }

    // Helpermethod

    
    private static Command testPath() {
    return swerve.defer(
            () -> {
                Pose2d current =
                        RobotStateRecorder.getPoseWorldRobotCurrent().toPose2d();
                List<Pose2d> waypoints = List.of(current, kTestB, kTestC);
                PathPlannerPath path =
                        generatePath(
                                waypoints, Collections.emptyList(), 4.2, 10.0, 0.0);
                return followPath(path);
            });
    }

    private static boolean isPitchStable() {
        return Math.abs(swerve.getPitchVelocityRadPerSec()) < 1.5;
    }

    private static double getRobotX() {
        return RobotStateRecorder.getPoseWorldRobotCurrent().toPose2d().getX();
    }

    private static boolean hasCrossedBump(boolean isToNeutral) {
        return isToNeutral
                ? AllianceFlipUtil.applyX(getRobotX())
                        > FieldConstants.LinesVertical.neutralZoneNear
                : AllianceFlipUtil.applyX(getRobotX()) < FieldConstants.LinesVertical.starting;
    }

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
