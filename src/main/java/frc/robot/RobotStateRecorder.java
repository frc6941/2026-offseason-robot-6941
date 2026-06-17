package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import lib.ironpulse.math.rbd.TransformRecorder;
import org.littletonrobotics.junction.Logger;

public class RobotStateRecorder extends TransformRecorder {
    public static final String kFrameShot = "Shot";

    public static final Translation3d kRobotToShot =
            new Translation3d(Meters.of(0), Meters.of(0), Meters.of(0.35));

    private static RobotStateRecorder instance;

    private RobotStateRecorder() {
        setBufferDuration(2.0);

        putTransform(kTransformWorldDriverStationBlue, kFrameWorld, kFrameDriverStationBlue);

        putTransform(kTransformWorldDriverStationRed, kFrameWorld, kFrameDriverStationRed);

        putTransform(new Pose3d(), Seconds.of(0.0), kFrameWorld, kFrameRobot);

        putTransform(
                new Pose3d(kRobotToShot, Rotation3d.kZero),
                Seconds.of(0.0),
                kFrameRobot,
                kFrameShot);
    }

    public static RobotStateRecorder getInstance() {
        if (instance == null) {
            instance = new RobotStateRecorder();
        }
        return instance;
    }

    public static void periodic() {
        Logger.recordOutput("RobotStateRecorder/poseWorldRobot", getPoseWorldRobotCurrent());

        Logger.recordOutput(
                "RobotStateRecorder/RobotRotation2d",
                getPoseWorldRobotCurrent().getRotation().toRotation2d());
    }

    public static Pose3d getPoseWorldRobotCurrent() {
        return getInstance()
                .getTransform(Seconds.of(Timer.getTimestamp()), kFrameWorld, kFrameRobot)
                .orElse(new Pose3d());
    }

    public static Pose3d getPoseWorldShotCurrent() {
        return getInstance()
                .getTransform(Seconds.of(Timer.getTimestamp()), kFrameWorld, kFrameShot)
                .orElse(new Pose3d());
    }
}
