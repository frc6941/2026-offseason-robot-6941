package frc.robot.subsystems.ShootingSubsystem;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.RobotStateRecorder;
import java.nio.file.Path;
import java.util.EnumMap;
import java.util.Map;

/**
 * Bridges the model table to a {@link ShotFrame}.
 *
 * <p>Workflow:
 *
 * <ol>
 *   <li>Pick a {@link TargetMode} (goal, feed-left, feed-right).
 *   <li>Read turret-to-target translation and goal-aligned robot velocity from
 *       {@link RobotStateRecorder}.
 *   <li>Interpolate the model table by distance and v_parallel.
 *   <li>Apply tuning (scale/offset/bias).
 *   <li>Solve turret yaw to cancel lateral velocity (v_perp).
 *   <li>Map model outputs to hardware with linear regression (RPM + hood angle).
 * </ol>
 *
 * <p>This skeleton always returns a {@link ShotFrame}; physical bounds and validity checks should
 * be applied elsewhere in the shooter subsystem.
 */
public class ShotCalculator {
    /** Targets supported by the shot calculator. */
    public enum TargetMode {
        GOAL,
        FEED_LEFT,
        FEED_RIGHT
    }

    /** Model-space outputs (exit speed, launch angle, flight time). */
    public record ShotModel(double exitSpeedMps, double launchAngleDeg, double flightTimeSec) {}

    private final Map<TargetMode, Path> modelJsonByTarget = new EnumMap<>(TargetMode.class);
    private TargetMode targetMode = TargetMode.GOAL;

    // Model tuning knobs (placeholders for NT-backed values)
    private double speedScale = 1.0;
    private double speedOffsetMps = 0.0;
    private double angleOffsetDeg = 0.0;
    private double trajectoryBiasDeg = 0.0;

    // Linear regression coefficients (placeholder)
    private double rpmA = 0.0;
    private double rpmB = 0.0;
    private double rpmC = 0.0;
    private double hoodA = 0.0;
    private double hoodB = 0.0;
    private double hoodC = 0.0;

    /** Loads model tables by target. */
    public void initialize(Map<TargetMode, Path> modelJsonByTarget) {
        this.modelJsonByTarget.clear();
        this.modelJsonByTarget.putAll(modelJsonByTarget);
        // TODO: parse JSON and build interpolation structures.
    }

    /** Selects which target table to use for interpolation. */
    public void setTargetMode(TargetMode mode) {
        this.targetMode = mode;
    }

    /** Refreshes tuning knobs (scale/offset/bias) from NetworkTables. */
    public void refreshTuningFromNetworkTables() {
        // TODO: read NT parameters and update tuning fields.
    }

    /**
     * Computes the shot frame for the current target mode.
     *
     * @param mode active target mode (goal/feed)
     * @param shotDelaySec look-ahead time; not used in skeleton
     */
    public ShotFrame computeShotFrame(TargetMode mode, double shotDelaySec) {
        if (mode != null) {
            setTargetMode(mode);
        }
        TargetMode activeMode = targetMode;
        Translation2d turretToTarget = getTurretToTargetTranslation(activeMode);
        double distanceMeters = Math.hypot(turretToTarget.getX(), turretToTarget.getY());
        Translation2d goalVelocity = getVelocityGoalRobotCurrent(activeMode);
        double vParallel = goalVelocity.getX();
        double vPerp = goalVelocity.getY();

        ShotModel model = lookupModel(distanceMeters, vParallel);
        model = applyModelTuning(model);

        double turretYawRad = solveTurretYaw(turretToTarget, vPerp, model);
        return mapToHardwareLinear(model, turretYawRad);
    }

    /**
     * Gets the turret-to-target translation for the given target mode.
     *
     * <p>For now, only GOAL is wired. Feed targets can be added once frames exist.
     */
    public Translation2d getTurretToTargetTranslation(TargetMode mode) {
        if (mode == TargetMode.GOAL) {
            return RobotStateRecorder.getTranslationTurretToGoalCurrent();
        }
        return RobotStateRecorder.getTranslationTurretToGoalCurrent();
    }

    /**
     * Gets the robot velocity in the goal-aligned frame (v_parallel, v_perp).
     *
     * <p>For now, only GOAL is wired. Feed targets can be added once frames exist.
     */
    public Translation2d getVelocityGoalRobotCurrent(TargetMode mode) {
        if (mode == TargetMode.GOAL) {
            return RobotStateRecorder.getVelocityGoalRobotCurrent();
        }
        return RobotStateRecorder.getVelocityGoalRobotCurrent();
    }

    /**
     * Looks up the model output using bilinear interpolation between distance and v_parallel
     * planes.
     *
     * <p>Skeleton returns zeros until the table is wired.
     */
    public ShotModel lookupModel(double distanceMeters, double velocityParallel) {
        return new ShotModel(0.0, 0.0, 0.0);
    }

    /** Applies tuning offsets in model space. */
    public ShotModel applyModelTuning(ShotModel model) {
        double speed = model.exitSpeedMps * speedScale + speedOffsetMps;
        double angle = model.launchAngleDeg + angleOffsetDeg + trajectoryBiasDeg;
        return new ShotModel(speed, angle, model.flightTimeSec);
    }

    /**
     * Solves turret yaw to cancel lateral velocity.
     *
     * <p>Skeleton currently aims directly at the target direction.
     */
    public double solveTurretYaw(Translation2d turretToTarget, double vPerp, ShotModel model) {
        return Math.atan2(turretToTarget.getY(), turretToTarget.getX());
    }

    /**
     * Maps model outputs to hardware using linear regression.
     *
     * <p>rpm = a*exitSpeed + b*launchAngle + c
     * <p>hood = a*exitSpeed + b*launchAngle + c
     */
    public ShotFrame mapToHardwareLinear(ShotModel model, double turretYawRad) {
        double rpm = rpmA * model.exitSpeedMps + rpmB * model.launchAngleDeg + rpmC;
        double hoodDeg = hoodA * model.exitSpeedMps + hoodB * model.launchAngleDeg + hoodC;

        Angle turretAngle = Radians.of(turretYawRad);
        Angle hoodAngle = Degrees.of(hoodDeg);
        AngularVelocity shooterVel = RotationsPerSecond.of(rpm / 60.0);
        return new ShotFrame(turretAngle, hoodAngle, shooterVel);
    }
}
