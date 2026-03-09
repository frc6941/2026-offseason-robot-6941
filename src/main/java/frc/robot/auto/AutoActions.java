package frc.robot.auto;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Meters;
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
import frc.robot.subsystems.Configs.IntakerExtensionParamsNT;
import frc.robot.subsystems.Configs.SwerveMK5Config;
import frc.robot.subsystems.IntakerSubsystem;
import frc.robot.subsystems.ShootingSubsystem.ShootingSuperstructure;
import frc.robot.subsystems.ShootingSubsystem.ShotCalculator;
import java.util.Collections;
import java.util.List;
import java.util.function.Supplier;
import lib.ironpulse.math.rbd.TransformRecorder;
import lib.ironpulse.swerve.Swerve;
import lib.ironpulse.swerve.SwerveCommands;
import lib.ironpulse.swerve.SwerveLimit;
import lib.ironpulse.swerve.commands.SwerveDriveToAllign;
import lib.ironpulse.swerve.commands.SwerveDriveToPose;
import lib.ironpulse.utils.AllianceFlipUtil;
import org.littletonrobotics.junction.Logger;

public class AutoActions {
    public static final double kVerticalSlopelineL = 5.59;
    public static final double kVerticalSlopelineR = 2.41;
    public static final Pose2d kSweepStartPoseL =
            new Pose2d(7.54, kVerticalSlopelineL, new Rotation2d(Degrees.of(0)));
    public static final Pose2d kSweepStartPoseR =
            new Pose2d(7.54, kVerticalSlopelineR, new Rotation2d(Degrees.of(0)));
    public static final Pose2d kSweepEndPoseL =
            new Pose2d(7.54, 5.79, new Rotation2d(Degrees.of(90)));
    public static final Pose2d kSweepEndPoseR =
            new Pose2d(7.54, 2.21, new Rotation2d(Degrees.of(-90)));

    public static final Pose2d kSlopeFrontL =
            new Pose2d(5.84, kVerticalSlopelineL, new Rotation2d(Degrees.of(-45)));
    public static final Pose2d kSlopeFrontR =
            new Pose2d(5.84, kVerticalSlopelineR, new Rotation2d(Degrees.of(45)));
    public static final Pose2d kSlopeEndL =
            new Pose2d(3.385, kVerticalSlopelineL, new Rotation2d(Degrees.of(180)));
    public static final Pose2d kSlopeEndR =
            new Pose2d(3.385, kVerticalSlopelineR, new Rotation2d(Degrees.of(180)));

    public static final Pose2d kStationIntake =
            new Pose2d(0.59, 0.66, new Rotation2d(Degrees.of(180)));

    public static final Pose2d kQuickSweepEndPoseL =
            new Pose2d(8.45, kVerticalSlopelineL, new Rotation2d(Degrees.of(0)));
    public static final Pose2d kQuickSweepEndPoseR =
            new Pose2d(8.45, kVerticalSlopelineR, new Rotation2d(Degrees.of(0)));

    public static final Pose2d kclimbL = new Pose2d(1.478, 4.136, new Rotation2d(Degrees.of(0)));
    public static final Pose2d kclimbR = new Pose2d(1.478, 3.331, new Rotation2d(Degrees.of(0)));

    public static final Pose2d kTestA = new Pose2d(1.509, 6.14, new Rotation2d(Degrees.of(6.3)));
    public static final Pose2d kTestB = new Pose2d(2.79, 5.2, new Rotation2d(Degrees.of(-72)));
    public static final Pose2d kTestC = new Pose2d(2.69, 3.07, new Rotation2d(Degrees.of(-110)));
    public static final Rotation2d kTestRotationA = new Rotation2d(Degrees.of(0));
    public static final double kTestRotationAPose = 1;
    public static final Rotation2d kTestRotationB = new Rotation2d(Degrees.of(45));
    public static final double kTestRotationBPose = 2;

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

    static Command drivePastSlope(boolean isLeft, boolean isToNeutral) {
        return swerve.defer(
                () -> {
                    Pose2d slopeFront =
                            AllianceFlipUtil.apply(isLeft ? kSlopeFrontL : kSlopeFrontR);
                    Pose2d slopeEnd = AllianceFlipUtil.apply(isLeft ? kSlopeEndL : kSlopeEndR);
                    Pose2d targetPose = isToNeutral ? slopeFront : slopeEnd;
                    return Commands.deadline(waitCrossedBump(isToNeutral), driveToPose(targetPose))
                            .beforeStarting(
                                    () ->
                                            Logger.recordOutput(
                                                    "Temp/isPitchStable", isPitchStable()));
                });
    }

    static Command waitCrossedBump(boolean isToNeutral) {
        return Commands.waitUntil(() -> isPitchStable() && hasCrossedBump(isToNeutral));
    }

    static Command allignToClimb(boolean isLeft) {
        return swerve.defer(
                () -> {
                    Pose2d climbPose = AllianceFlipUtil.apply(isLeft ? kclimbL : kclimbR);
                    return driveToAllign(climbPose, AutoActions::getShiftDirectionTowardBump);
                });
    }

    static Command allignToStation() {
        return swerve.defer(
                () ->
                        driveToAllign(
                                AllianceFlipUtil.apply(kStationIntake),
                                AutoActions::getShiftDirectionTowardBump));
    }

    public static boolean hasCrossedBump(boolean isToNeutral) {
        return isToNeutral
                ? AllianceFlipUtil.applyX(getRobotX())
                        > FieldConstants.LinesVertical.neutralZoneNear
                : AllianceFlipUtil.applyX(getRobotX()) < FieldConstants.LinesVertical.starting;
    }

    public static boolean isPitchStable() {
        return Math.abs(swerve.getPitchVelocityRadPerSec()) < 1.5;
    }

    static Command driveToPose(Supplier<Pose2d> targetPoseSupplier) {
        return swerve.defer(
                () -> {
                    Pose2d targetPose = targetPoseSupplier.get();
                    return new SwerveDriveToPose(
                                    swerve,
                                    () -> RobotStateRecorder.getPoseWorldRobotCurrent(),
                                    () -> new Pose3d(targetPose),
                                    () -> RobotStateRecorder.getVelocityWorldRobotCurrent(),
                                    new PIDController(
                                            AutoParamsNT.AutoPoseParams.kpStrave.getValue(),
                                            AutoParamsNT.AutoPoseParams.kiStrave.getValue(),
                                            AutoParamsNT.AutoPoseParams.kdStrave.getValue()),
                                    new PIDController(
                                            AutoParamsNT.AutoPoseParams.kpSpin.getValue(),
                                            AutoParamsNT.AutoPoseParams.kiSpin.getValue(),
                                            AutoParamsNT.AutoPoseParams.kdSpin.getValue()),
                                    Meters.of(
                                            AutoParamsNT.AutoPoseParams.tolerancePositionM
                                                    .getValue()),
                                    Degrees.of(
                                            AutoParamsNT.AutoPoseParams.toleranceHeadingDeg
                                                    .getValue()))
                            .beforeStarting(
                                    Commands.runOnce(
                                            () ->
                                                    Logger.recordOutput(
                                                            "Temp/targetPose", targetPose)));
                });
    }

    public static Command driveToPose(Pose2d targetPose) {
        return driveToPose(() -> targetPose);
    }

    private static Rotation2d getShiftDirectionTowardBump() {
        return AllianceFlipUtil.shouldFlip() ? Rotation2d.kPi : Rotation2d.kZero;
    }

    static Command driveToAllign(
            Pose2d targetPose, Supplier<Rotation2d> shiftingDirectionSupplier) {
        return new SwerveDriveToAllign(
                        swerve,
                        () -> RobotStateRecorder.getPoseWorldRobotCurrent(),
                        () -> RobotStateRecorder.getVelocityWorldRobotCurrent(),
                        () -> targetPose,
                        shiftingDirectionSupplier,
                        swerve.getSwerveLimit(),
                        1.2,
                        1.6)
                .beforeStarting(
                        Commands.runOnce(
                                () -> {
                                    Logger.recordOutput("Temp/targetAllignPose", targetPose);
                                    swerve.setSwerveModuleLimit(SwerveMK5Config.kAutoLimit);
                                }))
                .finallyDo(() -> swerve.setSwerveModuleLimitDefault());
    }

    public static Command followPath(PathPlannerPath path) {
        return new FollowPathCommand(
                        path,
                        () -> RobotStateRecorder.getPoseWorldRobotCurrent().toPose2d(),
                        swerve::getChassisSpeeds,
                        (vel, ff) -> {
                            swerve.runTwist(vel);
                            if (vel != null) {
                                Logger.recordOutput("Temp/", vel);
                            }
                            ;
                        },
                        new PPHolonomicDriveController(
                                new PIDConstants(
                                        AutoParamsNT.AutoPathParams.kpStrave.getValue(),
                                        AutoParamsNT.AutoPathParams.kiStrave.getValue(),
                                        AutoParamsNT.AutoPathParams.kdStrave.getValue()),
                                new PIDConstants(
                                        AutoParamsNT.AutoPathParams.kpSpin.getValue(),
                                        AutoParamsNT.AutoPathParams.kiSpin.getValue(),
                                        AutoParamsNT.AutoPathParams.kdSpin.getValue()),
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

    public static Command Intake() {
        return intake.runIntake();
    }

    public static Command runFeed() {
        return intake.runFeed();
    }

    public static Command oscillateIntakeFeed() {
        return Commands.defer(
                () -> {
                    double feedOscillationRateHz =
                            IntakerExtensionParamsNT.feedOscillationRateHz.getValue();
                    if (feedOscillationRateHz <= 0.0) {
                        return runFeed();
                    }

                    double halfCycleSeconds = 0.5 / feedOscillationRateHz;
                    return Commands.sequence(
                                    Intake(),
                                    Commands.waitSeconds(halfCycleSeconds),
                                    runFeed().withTimeout(halfCycleSeconds))
                            .repeatedly();
                },
                Collections.emptySet());
    }

    public static Command retractIntake() {
        return intake.runRetract();
    }

    public static Command shoot() {
        return shooter.shootWhenReady(false);
    }

    // Helpermethod

    public static Command testPath() {
        return swerve.defer(
                () -> {
                    Pose2d current = RobotStateRecorder.getPoseWorldRobotCurrent().toPose2d();
                    RotationTarget rotationTargetA =
                            new RotationTarget(kTestRotationAPose, kTestRotationA);
                    RotationTarget rotationTargetB =
                            new RotationTarget(kTestRotationBPose, kTestRotationB);
                    List<Pose2d> waypoints =
                            List.of(
                                    current,
                                    AllianceFlipUtil.apply(kTestA),
                                    AllianceFlipUtil.apply(kTestB),
                                    AllianceFlipUtil.apply(kTestC));
                    List<RotationTarget> rotationTargets =
                            List.of(rotationTargetA, rotationTargetB);
                    PathPlannerPath path =
                            generatePath(waypoints, Collections.emptyList(), 4.2, 6.0, 0.0);
                    return followPath(path);
                });
    }

    public static Command followPathFile(String pathName, boolean shouldMirror) {
        return swerve.defer(
                () -> {
                    PathPlannerPath path;
                    try {
                        path = PathPlannerPath.fromPathFile(pathName);
                    } catch (java.io.IOException | org.json.simple.parser.ParseException e) {
                        throw new RuntimeException("Failed to load path file: " + pathName, e);
                    }
                    if (AllianceFlipUtil.shouldFlip()) {
                        path = path.flipPath();
                        path = shouldMirror ? path.mirrorPath() : path;
                    }
                    return followPath(path);
                });
    }

    private static double getRobotX() {
        return RobotStateRecorder.getPoseWorldRobotCurrent().toPose2d().getX();
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
