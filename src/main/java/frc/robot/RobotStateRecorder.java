package frc.robot;

import static edu.wpi.first.units.Units.Meters;
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
import lib.ironpulse.math.rbd.TransformRecorder;
import org.littletonrobotics.junction.Logger;

public class RobotStateRecorder extends TransformRecorder {
    private static RobotStateRecorder instance;
    private static TimeInterpolatableBuffer<Pose2d> velocityRobotBuffer;

    public static final String kFrameTurret = "Turret";
    public static final String kFrameGoal = "Goal";
    public static final Translation3d kRobotToTurret =
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
                new Pose3d(kRobotToTurret, Rotation3d.kZero),
                Seconds.of(0.0),
                kFrameRobot,
                kFrameTurret); // dynamic TRobotTurret
        putTransform(
                new Pose3d(FieldConstants.Hub.innerCenterPoint, Rotation3d.kZero),
                Seconds.of(0.0),
                kFrameWorld,
                kFrameGoal); // static TWorldGoal (blue reference)
    }

    public static RobotStateRecorder getInstance() {
        if (instance == null) {
            instance = new RobotStateRecorder();
        }
        return instance;
    }

    public static void periodic() {
        Logger.recordOutput("RobotStateRecorder/fixed", new Pose3d());
        // logging
        Logger.recordOutput(
                "RobotStateRecorder/poseWorldRobot", RobotStateRecorder.getPoseWorldRobotCurrent());
        Logger.recordOutput(
                "RobotStateRecorder/velocityRobot", RobotStateRecorder.getVelocityRobotCurrent());
        Logger.recordOutput(
                "RobotStateRecorder/velocityWorldRobot",
                RobotStateRecorder.getVelocityWorldRobotCurrent());
        Logger.recordOutput(
                "RobotStateRecorder/poseTurret", RobotStateRecorder.getPoseWorldTurretCurrent());
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

    public static Pose3d getPoseWorldTurretCurrent() {
        return RobotStateRecorder.getInstance()
                .getTransform(
                        Seconds.of(Timer.getTimestamp()),
                        TransformRecorder.kFrameWorld,
                        RobotStateRecorder.kFrameTurret)
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

    public static Pose3d getPoseWorldGoalCurrent() {
        Pose3d goalBlue =
                RobotStateRecorder.getInstance()
                        .getTransform(
                                Seconds.of(Timer.getTimestamp()),
                                TransformRecorder.kFrameWorld,
                                RobotStateRecorder.kFrameGoal)
                        .orElse(new Pose3d());
        boolean isBlue =
                DriverStation.getAlliance()
                        .orElse(DriverStation.Alliance.Blue)
                        .equals(DriverStation.Alliance.Blue);
        if (isBlue) {
            return goalBlue;
        }
        return new Pose3d(
                new Translation3d(
                        FieldConstants.fieldLength - goalBlue.getX(),
                        goalBlue.getY(),
                        goalBlue.getZ()),
                goalBlue.getRotation());
    }

    public static Translation2d getTranslationTurretToGoalCurrent() {
        Pose3d turretPoseWorld = getPoseWorldTurretCurrent();
        Pose3d goalPoseWorld = getPoseWorldGoalCurrent();
        Translation3d delta =
                goalPoseWorld.getTranslation().minus(turretPoseWorld.getTranslation());
        return new Translation2d(delta.getX(), delta.getY());
    }

    public static Translation2d getVelocityGoalRobotCurrent() {
        Translation2d turretToGoal = getTranslationTurretToGoalCurrent();
        double dist = Math.hypot(turretToGoal.getX(), turretToGoal.getY());
        if (dist < 1e-6) {
            return new Translation2d();
        }
        Translation2d lineDir = new Translation2d(turretToGoal.getX() / dist, turretToGoal.getY() / dist);
        Translation2d velWorld = getVelocityWorldRobotCurrent().getTranslation();
        double vParallel = velWorld.getX() * lineDir.getX() + velWorld.getY() * lineDir.getY();
        double vPerp = velWorld.getX() * (-lineDir.getY()) + velWorld.getY() * lineDir.getX();
        return new Translation2d(vParallel, vPerp);
    }
}
