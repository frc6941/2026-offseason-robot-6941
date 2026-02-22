package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static lib.ironpulse.math.MathTools.toPose2d;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.ShootingSubsystem.ShotFrame;
import lib.ironpulse.math.rbd.TransformRecorder;
import lib.ironpulse.utils.AllianceFlipUtil;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

public class RobotStateRecorder extends TransformRecorder {
    private static RobotStateRecorder instance;
    private static TimeInterpolatableBuffer<Pose2d> velocityRobotBuffer;

    @Getter @Setter
    private static ShotFrame currentFrame =
            new ShotFrame(Degrees.of(0.0), Degrees.of(0.0), MetersPerSecond.of(0.0));

    @Getter @Setter
    private static ShotFrame cmdFrame =
            new ShotFrame(Degrees.of(0.0), Degrees.of(0.0), MetersPerSecond.of(0.0));

    public static final String kFrameShot = "Shot";
    public static final String kFrameGoal = "Goal";
    public static final String kFrameFeedUp = "FeedUp";
    public static final String kFrameFeedDown = "FeedDown";
    public static final Translation3d kRobotToShot =
            new Translation3d(Meters.of(0), Meters.of(0.0), Meters.of(0.35));

    private RobotStateRecorder() {
        setBufferDuration(2.0);
        velocityRobotBuffer = TimeInterpolatableBuffer.createBuffer(2.0);

        // add default transforms
        putTransform(
                kTransformWorldDriverStationBlue,
                kFrameWorld,
                kFrameDriverStationBlue); // static: TWorldDSB
        putTransform(
                kTransformWorldDriverStationRed,
                kFrameWorld,
                kFrameDriverStationRed); // static TWorldDSR
        putTransform(
                new Pose3d(),
                Seconds.of(0.0),
                kFrameWorld,
                kFrameRobot); // dynamic TWorldRobot at origin
        putTransform(
                new Pose3d(kRobotToShot, Rotation3d.kZero),
                Seconds.of(0.0),
                kFrameRobot,
                kFrameShot); // dynamic TRobotShot
        putTransform(
                new Pose3d(FieldConstants.Hub.innerCenterPoint, Rotation3d.kZero),
                Seconds.of(0.0),
                kFrameWorld,
                kFrameGoal); // static TWorldGoal (blue reference)

        putTransform(
                new Pose3d(0,8,0,new Rotation3d()),
                Seconds.of(0.0),
                kFrameWorld,
                kFrameFeedUp); // static TWorldFeedLeft
        putTransform(
                new Pose3d(),
                Seconds.of(0.0),
                kFrameWorld,
                kFrameFeedDown); // static TWorldFeedRight
    }

    public static RobotStateRecorder getInstance() {
        if (instance == null) {
            instance = new RobotStateRecorder();
        }
        return instance;
    }

    public static void periodic() {
        // logging
        Logger.recordOutput(
                "RobotStateRecorder/poseWorldRobot", RobotStateRecorder.getPoseWorldRobotCurrent());
        Logger.recordOutput(
                "RobotStateRecorder/velocityRobot", RobotStateRecorder.getVelocityRobotCurrent());
        Logger.recordOutput(
                "RobotStateRecorder/velocityWorldRobot",
                RobotStateRecorder.getVelocityWorldRobotCurrent());

        Logger.recordOutput(
                "RobotStateRecorder/ShotFrame/TargetPoseWorld",
                getPoseWorldTargetCurrent(kFrameGoal));
        Logger.recordOutput(
                "RobotStateRecorder/ShotFrame/poseShot",
                RobotStateRecorder.getPoseWorldShotCurrent());
        Logger.recordOutput(
                "RobotStateRecorder/ShotFrame/currentFrame/turretAngleWorldDeg",
                currentFrame.turretAngleWorld().in(Degrees));
        Logger.recordOutput(
                "RobotStateRecorder/ShotFrame/currentFrame/hoodAngleDeg",
                currentFrame.hoodAngle().in(Degrees));
        Logger.recordOutput(
                "RobotStateRecorder/ShotFrame/currentFrame/muzzleSpeedMps",
                currentFrame.muzzleSpeed().in(MetersPerSecond));
        Logger.recordOutput(
                "RobotStateRecorder/ShotFrame/cmdFrame/turretAngleWorldDeg",
                cmdFrame.turretAngleWorld().in(Degrees));
        Logger.recordOutput(
                "RobotStateRecorder/ShotFrame/cmdFrame/hoodAngleDeg",
                cmdFrame.hoodAngle().in(Degrees));
        Logger.recordOutput(
                "RobotStateRecorder/ShotFrame/cmdFrame/muzzleSpeedMps",
                cmdFrame.muzzleSpeed().in(MetersPerSecond));
    }

    public static void putVelocityRobot(Time time, ChassisSpeeds speed) {
        velocityRobotBuffer.addSample(time.in(Seconds), toPose2d(speed));
    }

    public static Pose2d getVelocityRobotCurrent() {
        return velocityRobotBuffer.getSample(Timer.getTimestamp()).orElse(new Pose2d());
    }

    public static Pose2d getVelocityWorldRobotCurrent() {
        // robot-relative velocity (dx, dy, dθ) and current robot pose in world
        Pose2d velocityRobot = getVelocityRobotCurrent();
        Pose3d poseWorldRobot = getPoseWorldRobotCurrent();

        // drop to 2D to get the robot's heading in the XY plane
        Pose2d pose2dWR = poseWorldRobot.toPose2d();
        Translation2d velRobotTrans = velocityRobot.getTranslation();

        // rotate the translational velocity by the robot’s heading
        Translation2d velWorldTrans = velRobotTrans.rotateBy(pose2dWR.getRotation());

        // preserve the same angular component
        return new Pose2d(velWorldTrans, velocityRobot.getRotation());
    }

    public static Pose3d getPoseWorldRobotCurrent() {
        return RobotStateRecorder.getInstance()
                .getTransform(
                        Seconds.of(Timer.getTimestamp()),
                        TransformRecorder.kFrameWorld,
                        TransformRecorder.kFrameRobot)
                .orElse(new Pose3d());
    }

    public static Pose3d getPoseWorldShotCurrent() {
        return RobotStateRecorder.getInstance()
                .getTransform(
                        Seconds.of(Timer.getTimestamp()),
                        TransformRecorder.kFrameWorld,
                        RobotStateRecorder.kFrameShot)
                .orElse(new Pose3d());
    }

    public static Pose3d getPoseDriverRobotCurrent() {
        return RobotStateRecorder.getInstance()
                .getTransform(
                        Seconds.of(Timer.getTimestamp()),
                        DriverStation.getAlliance()
                                        .orElse(DriverStation.Alliance.Blue)
                                        .equals(DriverStation.Alliance.Blue)
                                ? RobotStateRecorder.kFrameDriverStationBlue
                                : RobotStateRecorder.kFrameDriverStationRed,
                        TransformRecorder.kFrameRobot)
                .orElse(new Pose3d());
    }

    public static Pose3d getPoseWorldTargetCurrent(String targetFrame) {
        Pose3d poseBlue =
                RobotStateRecorder.getInstance()
                        .getTransform(
                                Seconds.of(Timer.getTimestamp()),
                                TransformRecorder.kFrameWorld,
                                targetFrame)
                        .orElse(new Pose3d());
        return AllianceFlipUtil.apply(poseBlue);
    }

    public static Translation2d getTranslationShotToTargetCurrent(String targetFrame) {
        Pose3d shotPoseWorld = getPoseWorldShotCurrent();
        Pose3d targetPoseWorld = getPoseWorldTargetCurrent(targetFrame);
        Translation3d delta =
                targetPoseWorld.getTranslation().minus(shotPoseWorld.getTranslation());
        return delta.toTranslation2d();
    }

    public static Translation2d getVelocityTargetRobotCurrent(String targetFrame) {
        Translation2d shotToTarget = getTranslationShotToTargetCurrent(targetFrame);
        Translation2d velWorld = getVelocityWorldRobotCurrent().getTranslation();
        Translation2d velTarget =
                velWorld.rotateBy(shotToTarget.getAngle().unaryMinus()); // +X is toward target
        return velTarget;
    }
}
