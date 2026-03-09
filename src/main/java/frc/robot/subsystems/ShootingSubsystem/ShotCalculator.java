package frc.robot.subsystems.ShootingSubsystem;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.fasterxml.jackson.core.type.TypeReference;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import frc.robot.FieldConstants;
import frc.robot.RobotConstants;
import frc.robot.RobotStateRecorder;
import frc.robot.subsystems.Configs.ShotCalculatorParamsNT;
import java.io.IOException;
import java.nio.file.Path;
import java.util.EnumMap;
import java.util.List;
import java.util.Map;
import java.util.NavigableMap;
import java.util.TreeMap;
import org.littletonrobotics.junction.Logger;

/**
 * Bridges the model table to a {@link ShotFrame}.
 *
 * <p>Workflow:
 *
 * <ol>
 *   <li>Pick a {@link TargetMode} (goal, feed-left, feed-right).
 *   <li>Read turret-to-target translation and goal-aligned robot velocity from {@link
 *       RobotStateRecorder}.
 *   <li>Interpolate the model table by distance and v_parallel.
 *   <li>Apply tuning (scale/offset/bias).
 *   <li>Solve turret yaw to cancel lateral velocity (v_perp).
 *   <li>Return a model-space {@link ShotFrame} (muzzle speed + launch angle).
 * </ol>
 *
 * <p>{@link ShotFrame} is kept in model units:
 *
 * <ul>
 *   <li>turretAngleWorld: world yaw to target
 *   <li>hoodAngle: launch angle from the model (deg)
 *   <li>muzzleSpeed: exit speed from the model (m/s)
 * </ul>
 *
 * <p>Calibration from model units to actuator units (RPM + BBA angle) is applied in the shooter
 * subsystem using {@code ShotCalculatorParams}.
 */
public class ShotCalculator {
    private final Map<TargetMode, Path> modelJsonByTarget = new EnumMap<>(TargetMode.class);
    private final Map<TargetMode, NavigableMap<Double, NavigableMap<Double, ShotModel>>>
            tableByTarget = new EnumMap<>(TargetMode.class);

    private static NavigableMap<Double, NavigableMap<Double, ShotModel>> buildTable(
            List<ModelPoint> points) {
        NavigableMap<Double, NavigableMap<Double, ShotModel>> table = new TreeMap<>();
        for (ModelPoint p : points) {
            table.computeIfAbsent(p.distance, k -> new TreeMap<>())
                    .put(p.robot_vel, new ShotModel(p.final_vel, p.final_angle, p.flight_time));
        }
        return table;
    }

    private static ShotModel interpolateVelocityPlane(
            NavigableMap<Double, ShotModel> plane, double velocityParallel) {
        if (plane.isEmpty()) {
            return new ShotModel(0.0, 0.0, 0.0);
        }
        Map.Entry<Double, ShotModel> v0 = plane.floorEntry(velocityParallel);
        Map.Entry<Double, ShotModel> v1 = plane.ceilingEntry(velocityParallel);
        if (v0 == null) {
            v0 = plane.firstEntry();
        }
        if (v1 == null) {
            v1 = plane.lastEntry();
        }
        if (v0.getKey().equals(v1.getKey())) {
            return v0.getValue();
        }
        double t = (velocityParallel - v0.getKey()) / (v1.getKey() - v0.getKey());
        return interpolateModel(v0.getValue(), v1.getValue(), t);
    }

    private static ShotModel interpolateModel(ShotModel a, ShotModel b, double t) {
        double speed = a.exitSpeedMps + (b.exitSpeedMps - a.exitSpeedMps) * t;
        double angle = a.launchAngleDeg + (b.launchAngleDeg - a.launchAngleDeg) * t;
        double time = a.flightTimeSec + (b.flightTimeSec - a.flightTimeSec) * t;
        return new ShotModel(speed, angle, time);
    }

    /** Loads model tables by target. */
    public void initialize(Map<TargetMode, Path> modelJsonByTarget) {
        this.modelJsonByTarget.clear();
        this.modelJsonByTarget.putAll(modelJsonByTarget);
        this.tableByTarget.clear();

        ObjectMapper mapper = new ObjectMapper();
        for (Map.Entry<TargetMode, Path> entry : this.modelJsonByTarget.entrySet()) {
            TargetMode mode = entry.getKey();
            Path path = entry.getValue();
            try {
                List<ModelPoint> points =
                        mapper.readValue(path.toFile(), new TypeReference<List<ModelPoint>>() {});
                tableByTarget.put(mode, buildTable(points));
            } catch (IOException e) {
                throw new RuntimeException("Failed to load model table: " + path, e);
            }
        }
    }

    /** Computes the shot frame using automatic zone-based shot decision. */
    public ShotFrame computeShotFrame() {
        TargetMode mode = decideShotMode();
        String targetFrame;
        if (mode == TargetMode.GOAL) {
            targetFrame = RobotStateRecorder.kFrameGoal;
        } else {
            double yAlliance = RobotStateRecorder.getPoseDriverRobotCurrent().getY();
            targetFrame =
                    yAlliance > FieldConstants.fieldWidth / 2.0
                            ? RobotStateRecorder.kFrameFeedUp
                            : RobotStateRecorder.kFrameFeedDown;
        }
        return computeShotFrame(mode, targetFrame);
    }

    public TargetMode decideShotMode() {
        double xAlliance = RobotStateRecorder.getPoseDriverRobotCurrent().getX();
        if (xAlliance <= FieldConstants.LinesVertical.neutralZoneNear) {
            return TargetMode.GOAL;
        }
        return TargetMode.FEED;
    }

    /** Computes the shot frame for a forced model mode and forced target frame. */
    public ShotFrame computeShotFrame(TargetMode mode, String targetFrame) {
        RobotStateRecorder.setKFrameTarget(targetFrame);
        Translation2d shotToTargetCurrent =
                RobotStateRecorder.getTranslationShotToTargetCurrent(targetFrame);
        Translation2d velocityWorldRobotCurrent =
                RobotStateRecorder.getVelocityWorldRobotCmdCurrent().getTranslation();

        double currentDistanceMeters = shotToTargetCurrent.getNorm();
        Translation2d currentTargetVelocity =
                velocityWorldRobotCurrent.rotateBy(shotToTargetCurrent.getAngle().unaryMinus());
        double currentVParallel = currentTargetVelocity.getX();

        ShotModel initialModel = lookupModel(currentDistanceMeters, currentVParallel, mode);
        double totalCyclesRaw =
                ShotCalculatorParamsNT.lookfwdDelayCycles.getValue()
                        + ShotCalculatorParamsNT.loookfwdDistanceScale.getValue()
                                * currentDistanceMeters
                        + ShotCalculatorParamsNT.lookfwdFlightScale.getValue()
                                * (initialModel.flightTimeSec / RobotConstants.LOOPER_DT);

        double totalCycles =
                MathUtil.clamp(
                        totalCyclesRaw,
                        ShotCalculatorParamsNT.lookfwdMinCycles.getValue(),
                        ShotCalculatorParamsNT.lookfwdMaxCycles.getValue());

        Translation2d shotToTargetPredicted =
                calculateLookfwdPose(shotToTargetCurrent, velocityWorldRobotCurrent, totalCycles);
        double distanceMeters = shotToTargetPredicted.getNorm();
        Translation2d targetVelocityPredicted =
                velocityWorldRobotCurrent.rotateBy(shotToTargetPredicted.getAngle().unaryMinus());
        double vParallel = targetVelocityPredicted.getX();
        double vPerp = targetVelocityPredicted.getY();

        Translation2d robotDelta =
                velocityWorldRobotCurrent.times(totalCycles * RobotConstants.LOOPER_DT);
        Pose3d shotPoseWorldCurrent = RobotStateRecorder.getPoseWorldShotCurrent();
        Pose3d shotPoseWorldPredicted =
                new Pose3d(
                        shotPoseWorldCurrent.getTranslation().plus(new Translation3d(robotDelta)),
                        shotPoseWorldCurrent.getRotation());
        Logger.recordOutput("ShotCalculator/lookfwd/totalCyclesRaw", totalCyclesRaw);
        Logger.recordOutput("ShotCalculator/lookfwd/totalCycles", totalCycles);
        Logger.recordOutput(
                "ShotCalculator/lookfwd/shotPoseWorldPredicted", shotPoseWorldPredicted);

        ShotModel model = lookupModel(distanceMeters, vParallel, mode);
        model = applyModelTuning(model, mode);

        Angle turretYawRad = solveTurretYaw(shotToTargetPredicted, vPerp, model);
        return new ShotFrame(
                turretYawRad,
                Degrees.of(90).minus(Degrees.of(model.launchAngleDeg)),
                MetersPerSecond.of(model.exitSpeedMps));
    }

    public Translation2d calculateLookfwdPose(
            Translation2d shotToTargetCurrent,
            Translation2d velocityWorldRobotCurrent,
            double totalCycles) {
        Translation2d robotDelta =
                velocityWorldRobotCurrent.times(totalCycles * RobotConstants.LOOPER_DT);
        return shotToTargetCurrent.minus(robotDelta);
    }

    /**
     * Looks up the model output using bilinear interpolation between distance and v_parallel
     * planes.
     *
     * <p>Expected JSON fields (units):
     *
     * <ul>
     *   <li>distance (m)
     *   <li>robot_vel (m/s)
     *   <li>final_vel (m/s)
     *   <li>final_angle (deg)
     * </ul>
     */
    public ShotModel lookupModel(double distanceMeters, double velocityParallel, TargetMode mode) {
        NavigableMap<Double, NavigableMap<Double, ShotModel>> table = tableByTarget.get(mode);
        if (table == null || table.isEmpty()) {
            return new ShotModel(0.0, 0.0, 0.0);
        }

        Map.Entry<Double, NavigableMap<Double, ShotModel>> distFloor =
                table.floorEntry(distanceMeters);
        Map.Entry<Double, NavigableMap<Double, ShotModel>> distCeil =
                table.ceilingEntry(distanceMeters);
        if (distFloor == null) {
            distFloor = table.firstEntry();
        }
        if (distCeil == null) {
            distCeil = table.lastEntry();
        }

        double d0 = distFloor.getKey();
        double d1 = distCeil.getKey();
        ShotModel m0 = interpolateVelocityPlane(distFloor.getValue(), velocityParallel);
        if (d0 == d1) {
            return m0;
        }
        ShotModel m1 = interpolateVelocityPlane(distCeil.getValue(), velocityParallel);
        double t = (distanceMeters - d0) / (d1 - d0);
        return interpolateModel(m0, m1, t);
    }

    /** Applies tuning offsets in model space. */
    public ShotModel applyModelTuning(ShotModel model, TargetMode mode) {
        double speed = model.exitSpeedMps;
        double angle =
                mode == TargetMode.GOAL
                        ? model.launchAngleDeg
                                + ShotCalculatorParamsNT.trajectoryBiasDegGOAL.getValue()
                        : model.launchAngleDeg
                                + ShotCalculatorParamsNT.trajectoryBiasDegFEED.getValue();
        return new ShotModel(speed, angle, model.flightTimeSec);
    }

    /**
     * Solves turret yaw to cancel lateral velocity.
     *
     * <p>Compensates lateral velocity by yawing into the motion so the net lateral component is
     * near zero in the goal-aligned frame.
     */
    public Angle solveTurretYaw(Translation2d turretToTarget, double vPerp, ShotModel model) {
        Rotation2d baseYaw = turretToTarget.getAngle();
        double launchRad = Math.toRadians(model.launchAngleDeg);
        double vHoriz = model.exitSpeedMps * Math.cos(launchRad);
        if (Math.abs(vHoriz) < 1e-6) {
            return baseYaw.getMeasure();
        }
        double scaledVPerp = vPerp * ShotCalculatorParamsNT.lateralVelocityCompScale.getValue();
        double ratio = MathUtil.clamp(-scaledVPerp / vHoriz, -1.0, 1.0);
        Rotation2d yawComp = Rotation2d.fromRadians(Math.asin(ratio));
        return baseYaw.plus(yawComp).getMeasure();
    }

    /** Targets supported by the shot calculator. */
    public enum TargetMode {
        GOAL,
        FEED
    }

    /** Model-space outputs (exit speed, launch angle, flight time). */
    public record ShotModel(double exitSpeedMps, double launchAngleDeg, double flightTimeSec) {}

    private record ModelPoint(
            double distance,
            double robot_vel,
            double final_vel,
            double final_angle,
            double flight_time) {}
}
