package frc.robot.subsystems.ShootingSubsystem;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.fasterxml.jackson.core.type.TypeReference;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import frc.robot.RobotStateRecorder;
import frc.robot.subsystems.Configs.ShotCalculatorParamsNT;
import java.io.IOException;
import java.nio.file.Path;
import java.util.EnumMap;
import java.util.List;
import java.util.Map;
import java.util.NavigableMap;
import java.util.TreeMap;

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
    /** Targets supported by the shot calculator. */
    public enum TargetMode {
        GOAL,
        FEED_UP,
        FEED_DOWN
    }

    /** Model-space outputs (exit speed, launch angle, flight time). */
    public record ShotModel(double exitSpeedMps, double launchAngleDeg, double flightTimeSec) {}

    private record ModelPoint(
            double distance, double robot_vel, double final_vel, double final_angle) {}

    private final Map<TargetMode, Path> modelJsonByTarget = new EnumMap<>(TargetMode.class);
    private final Map<TargetMode, NavigableMap<Double, NavigableMap<Double, ShotModel>>>
            tableByTarget = new EnumMap<>(TargetMode.class);
    private TargetMode targetMode = TargetMode.GOAL;

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

    /**
     * Computes the shot frame for the current target mode.
     *
     * @param mode active target mode (goal/feed)
     */
    public ShotFrame computeShotFrame(TargetMode mode) {
        if (mode != null) {
            this.targetMode = mode;
        }else this.targetMode = TargetMode.GOAL;
        String targetFrame = 
            switch (this.targetMode) {
                case GOAL -> RobotStateRecorder.kFrameGoal;
                case FEED_UP -> RobotStateRecorder.kFrameFeedUp;
                case FEED_DOWN -> RobotStateRecorder.kFrameFeedDown;
        };
        Translation2d turretToTarget =
                RobotStateRecorder.getTranslationShotToTargetCurrent(targetFrame);
        double distanceMeters = turretToTarget.getNorm();
        Translation2d targetVelocity =
                RobotStateRecorder.getVelocityTargetRobotCurrent(targetFrame);
        double vParallel = targetVelocity.getX();
        double vPerp = targetVelocity.getY();

        ShotModel model = lookupModel(distanceMeters, vParallel);
        model = applyModelTuning(model);

        Angle turretYawRad = solveTurretYaw(turretToTarget, vPerp, model);
        return new ShotFrame(
                turretYawRad,
                Degrees.of(90).minus(Degrees.of(model.launchAngleDeg)),
                MetersPerSecond.of(model.exitSpeedMps));
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
    public ShotModel lookupModel(double distanceMeters, double velocityParallel) {
        NavigableMap<Double, NavigableMap<Double, ShotModel>> table = tableByTarget.get(targetMode);
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
    public ShotModel applyModelTuning(ShotModel model) {
        double speed =
                model.exitSpeedMps * ShotCalculatorParamsNT.speedScale.getValue()
                        + ShotCalculatorParamsNT.speedOffsetMps.getValue();
        double angle =
                model.launchAngleDeg
                        + ShotCalculatorParamsNT.angleOffsetDeg.getValue()
                        + ShotCalculatorParamsNT.trajectoryBiasDeg.getValue();
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

    private static NavigableMap<Double, NavigableMap<Double, ShotModel>> buildTable(
            List<ModelPoint> points) {
        NavigableMap<Double, NavigableMap<Double, ShotModel>> table = new TreeMap<>();
        for (ModelPoint p : points) {
            table.computeIfAbsent(p.distance, k -> new TreeMap<>())
                    .put(p.robot_vel, new ShotModel(p.final_vel, p.final_angle, 0.0));
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
}
