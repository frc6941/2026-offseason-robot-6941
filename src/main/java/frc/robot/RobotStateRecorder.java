package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static lib.ironpulse.math.MathTools.toPose2d;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.ShootingSubsystem.ShotFrame;
import lib.ironpulse.math.obstacle.Obstacle2d;
import lib.ironpulse.math.obstacle.PolygonObstacle2d;
import lib.ironpulse.math.rbd.TransformRecorder;
import lib.ironpulse.utils.AllianceFlipUtil;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class RobotStateRecorder extends TransformRecorder {
    public static final String kFrameShot = "Shot";
    public static final String kFrameGoal = "Goal";
    public static final String kFrameFeedUp = "FeedUp";
    public static final String kFrameFeedDown = "FeedDown";
    public static final Translation3d kRobotToShot =
            new Translation3d(Meters.of(0), Meters.of(0.0), Meters.of(0.35));
    private static RobotStateRecorder instance;
    private static TimeInterpolatableBuffer<Pose2d> velocityRobotBuffer;
    private static TimeInterpolatableBuffer<Pose2d> velocityRobotCmdBuffer;
    private static AngularVelocity omegaRobotCurrent = RadiansPerSecond.zero();

    @Getter @Setter
    private static ShotFrame currentFrame =
            new ShotFrame(Degrees.of(0.0), Degrees.of(0.0), MetersPerSecond.of(0.0));

    @Getter @Setter
    private static ShotFrame cmdFrame =
            new ShotFrame(Degrees.of(0.0), Degrees.of(0.0), MetersPerSecond.of(0.0));

    @Getter
    @Setter
    @AutoLogOutput(key = "RobotStateRecorder/kFrameTarget")
    private static String kFrameTarget = kFrameGoal;

    private RobotStateRecorder() {
        setBufferDuration(2.0);
        velocityRobotBuffer = TimeInterpolatableBuffer.createBuffer(2.0);
        velocityRobotCmdBuffer = TimeInterpolatableBuffer.createBuffer(2.0);
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
                new Pose3d(1.6, 7, 0, new Rotation3d()),
                Seconds.of(0.0),
                kFrameWorld,
                kFrameFeedUp);
        putTransform(
                new Pose3d(1.6, 1, 0, new Rotation3d()),
                Seconds.of(0.0),
                kFrameWorld,
                kFrameFeedDown);
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
                "RobotStateRecorder/RobotRotation2d",
                RobotStateRecorder.getPoseWorldRobotCurrent().getRotation().toRotation2d());
        Logger.recordOutput(
                "RobotStateRecorder/velocityWorldRobot",
                RobotStateRecorder.getVelocityWorldRobotCurrent());
        Logger.recordOutput(
                "RobotStateRecorder/velocityRobotCmd",
                RobotStateRecorder.getVelocityRobotCmdCurrent());
        Logger.recordOutput(
                "RobotStateRecorder/velocityWorldRobotCmd",
                RobotStateRecorder.getVelocityWorldRobotCmdCurrent());
        Logger.recordOutput(
                "RobotStateRecorder/omegaRobotCurrentRadPerSec",
                RobotStateRecorder.getOmegaRobotCurrent().in(RadiansPerSecond));
        Logger.recordOutput(
                "RobotStateRecorder/TargetPoseWorld", getPoseWorldTargetCurrent(kFrameTarget));
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
        Logger.recordOutput(
                "RobotStateRecorder/ShotFrame/Distance",
                getTranslationShotToTargetCurrent(kFrameTarget).getNorm());
        SmartDashboard.putNumber(
                "RobotStateRecorder/ShotFrame/cmdFrame/muzzleSpeedMps",
                cmdFrame.muzzleSpeed().in(MetersPerSecond));
    }

    public static final Obstacle2d towerZoneBlue =
            new PolygonObstacle2d(
                    new Translation2d(
                            0,
                            FieldConstants.Tower.centerPoint.getY()
                                    - FieldConstants.Tower.width / 2.0),
                    new Translation2d(
                            FieldConstants.Tower.frontFaceX,
                            FieldConstants.Tower.centerPoint.getY()
                                    - FieldConstants.Tower.width / 2.0),
                    new Translation2d(
                            FieldConstants.Tower.frontFaceX,
                            FieldConstants.Tower.centerPoint.getY()
                                    + FieldConstants.Tower.width / 2.0),
                    new Translation2d(
                            0,
                            FieldConstants.Tower.centerPoint.getY()
                                    + FieldConstants.Tower.width / 2.0));
    public static final Obstacle2d towerZoneRed =
            new PolygonObstacle2d(
                    new Translation2d(
                            FieldConstants.fieldLength - FieldConstants.Tower.frontFaceX,
                            FieldConstants.Tower.oppCenterPoint.getY()
                                    - FieldConstants.Tower.width / 2.0),
                    new Translation2d(
                            FieldConstants.fieldLength,
                            FieldConstants.Tower.oppCenterPoint.getY()
                                    - FieldConstants.Tower.width / 2.0),
                    new Translation2d(
                            FieldConstants.fieldLength,
                            FieldConstants.Tower.oppCenterPoint.getY()
                                    + FieldConstants.Tower.width / 2.0),
                    new Translation2d(
                            FieldConstants.fieldLength - FieldConstants.Tower.frontFaceX,
                            FieldConstants.Tower.oppCenterPoint.getY()
                                    + FieldConstants.Tower.width / 2.0));

    // Hub shooting zones
    public static final Obstacle2d hubZoneBlue =
            new PolygonObstacle2d(
                    new Translation2d(
                            5.0,
                            FieldConstants.Hub.topCenterPoint.getY()
                                    - FieldConstants.Hub.width / 2.0),
                    new Translation2d(
                            7.3,
                            FieldConstants.Hub.topCenterPoint.getY()
                                    - FieldConstants.Hub.width / 2.0),
                    new Translation2d(
                            7.3,
                            FieldConstants.Hub.topCenterPoint.getY()
                                    + FieldConstants.Hub.width / 2.0),
                    new Translation2d(
                            5.0,
                            FieldConstants.Hub.topCenterPoint.getY()
                                    + FieldConstants.Hub.width / 2.0));
    public static final Obstacle2d hubZoneRed =
            new PolygonObstacle2d(
                    new Translation2d(
                            FieldConstants.fieldLength - 7.3,
                            FieldConstants.Hub.oppTopCenterPoint.getY()
                                    - FieldConstants.Hub.width / 2.0),
                    new Translation2d(
                            FieldConstants.fieldLength - 5.0,
                            FieldConstants.Hub.oppTopCenterPoint.getY()
                                    - FieldConstants.Hub.width / 2.0),
                    new Translation2d(
                            FieldConstants.fieldLength - 5.0,
                            FieldConstants.Hub.oppTopCenterPoint.getY()
                                    + FieldConstants.Hub.width / 2.0),
                    new Translation2d(
                            FieldConstants.fieldLength - 7.3,
                            FieldConstants.Hub.oppTopCenterPoint.getY()
                                    + FieldConstants.Hub.width / 2.0));

    public static void putVelocityRobot(Time time, ChassisSpeeds speed) {
        velocityRobotBuffer.addSample(time.in(Seconds), toPose2d(speed));
    }

    public static void putVelocityRobotCmd(Time time, ChassisSpeeds speedCmd) {
        velocityRobotCmdBuffer.addSample(time.in(Seconds), toPose2d(speedCmd));
    }

    public static void putOmegaRobotCurrent(AngularVelocity omega) {
        omegaRobotCurrent = omega;
    }

    public static Pose2d getVelocityRobotCurrent() {
        return velocityRobotBuffer.getSample(Timer.getTimestamp()).orElse(new Pose2d());
    }

    public static AngularVelocity getOmegaRobotCurrent() {
        return omegaRobotCurrent;
    }

    public static Pose2d getVelocityRobotCmdCurrent() {
        return velocityRobotCmdBuffer.getSample(Timer.getTimestamp()).orElse(new Pose2d());
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

    public static Pose2d getVelocityWorldRobotCmdCurrent() {
        // robot-relative command velocity (dx, dy, dθ) and current robot pose in world
        Pose2d velocityRobotCmd = getVelocityRobotCmdCurrent();
        Pose3d poseWorldRobot = getPoseWorldRobotCurrent();

        // drop to 2D to get the robot's heading in the XY plane
        Pose2d pose2dWR = poseWorldRobot.toPose2d();
        Translation2d velRobotCmdTrans = velocityRobotCmd.getTranslation();

        // rotate the translational velocity by the robot’s heading
        Translation2d velWorldCmdTrans = velRobotCmdTrans.rotateBy(pose2dWR.getRotation());

        // preserve the same angular component
        return new Pose2d(velWorldCmdTrans, velocityRobotCmd.getRotation());
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
}
