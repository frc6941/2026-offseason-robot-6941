package frc.robot.subsystems.ShootingSubsystem;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import frc.robot.RobotStateRecorder;
import org.littletonrobotics.junction.Logger;

/**
 * Corrects a raw {@link ShotFrame} for robot pitch, roll, and shooter height deviation.
 *
 * <p>The shot model ({@code model.py}) was generated assuming a <b>level robot</b> with the
 * shooter at exactly {@value #NOMINAL_SHOOTER_HEIGHT_M} m. This class applies a closed-form
 * correction so that the ball's world-frame trajectory matches the model's intent, without
 * modifying the model tables or {@link ShotCalculator}.
 *
 * <p>Robot pose (pitch, roll, yaw) is read directly from {@link RobotStateRecorder#getPoseWorldRobotCurrent()}.
 * Actual shooter height is derived from {@link RobotStateRecorder#getPoseWorldShotCurrent()},
 * accounting for robot elevation on uneven surfaces.
 *
 * <h2>Coordinate Conventions</h2>
 *
 * <ul>
 *   <li><b>pitch α</b> – nose-up positive, {@link RobotStateRecorder#getRobotPitchRad()}
 *   <li><b>roll φ</b> – right-side-down positive, {@link RobotStateRecorder#getRobotRollRad()}
 *   <li><b>ψ_rel</b> – turret angle relative to robot forward = turretAngleWorld − robotYaw
 *   <li><b>Δh</b> – deviation of shot-frame world-Z from nominal; positive when elevated
 * </ul>
 *
 * <h2>Core Equation</h2>
 *
 * <p>Requiring the world-frame vertical launch-velocity component to equal what the model
 * calibrated leads to:
 *
 * <pre>
 *   A · sin(θ_cmd) + B · cos(θ_cmd) = C
 *
 *   A = cos(α) · cos(φ)
 *   B = sin(α)·cos(ψ_rel)  +  cos(α)·sin(φ)·sin(ψ_rel)
 *   C = sin(θ_model)  −  Δh / (v₀ · t_flight)
 *
 *   Closed-form solution:
 *     R      = √(A² + B²)
 *     θ_cmd  = asin(C / R) − atan2(B, A)
 * </pre>
 *
 * <h2>Lateral-Drift Correction (Yaw)</h2>
 *
 * <p>World-frame gravity is always (0, 0, −g) regardless of robot orientation — it does NOT
 * create a transverse component. The lateral drift comes entirely from the <b>launch direction
 * being geometrically tilted</b> by the same Ry(−α)·Rx(φ) rotation. Taking the world-Y
 * component perpendicular to the aim direction and expanding to first order:
 *
 * <pre>
 *   v_lateral ≈ v₀ · sin(θ) · (α·sin(ψ_rel) − φ·cos(ψ_rel))
 *
 *   δψ = asin( tan(θ_model) · (sin(φ)·cos(ψ_rel) − sin(α)·sin(ψ_rel)) )
 * </pre>
 *
 * <p>Note: pitch also contributes when the turret faces sideways (the sin(α)·sin(ψ_rel) term).
 */
public final class ShotPoseCompensator {

    /**
     * Shooter height assumed by model.py (m).
     * Δh is computed relative to this value using the world-Z of the shot frame.
     */
    public static final double NOMINAL_SHOOTER_HEIGHT_M = 0.63;

    /** Allowable commanded launch angle range (degrees). */
    private static final double THETA_MIN_DEG = 30.0;
    private static final double THETA_MAX_DEG = 89.0;

    private ShotPoseCompensator() {}

    // ─────────────────────────────────────────────────────────────────────────
    // Public API
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * Full compensation using the pre-looked-up {@link ShotCalculator.ShotModel}.
     *
     * <p>Preferred overload: uses the accurate flight time from the model table (which includes
     * drag and Magnus effects). All robot-pose inputs are read from {@link RobotStateRecorder}.
     *
     * <p>Usage example:
     *
     * <pre>{@code
     * ShotCalculator.ShotModel model = shotCalculator.lookupModel(distanceM, vParallel, mode);
     * ShotFrame raw   = shotCalculator.computeShotFrame(mode, targetFrame);
     * ShotFrame fixed = ShotPoseCompensator.compensate(raw, model);
     * RobotStateRecorder.setCmdFrame(fixed);
     * }</pre>
     *
     * @param raw   the frame produced by {@link ShotCalculator#computeShotFrame}
     * @param model the interpolated model entry for the current shot geometry
     * @return corrected {@link ShotFrame} ready to pass to {@code RobotStateRecorder.setCmdFrame}
     */
    public static ShotFrame compensate(ShotFrame raw, ShotCalculator.ShotModel model) {
        return applyCorrection(
                raw,
                model.exitSpeedMps(),
                model.flightTimeSec(),
                RobotStateRecorder.getPoseWorldRobotCurrent().getRotation().getY(),
                RobotStateRecorder.getPoseWorldRobotCurrent().getRotation().getX(),
                RobotStateRecorder.getPoseWorldRobotCurrent().getRotation().getZ(),
                readShooterHeightM());
    }

    /**
     * Convenience overload that approximates flight time from horizontal distance.
     *
     * <p>Use when the {@link ShotCalculator.ShotModel} is not readily available. Ignoring drag
     * introduces ~10–20 % error on the height term, but pitch/roll corrections remain accurate.
     *
     * @param raw        the frame produced by {@link ShotCalculator#computeShotFrame}
     * @param distanceM  horizontal distance from shooter to target (m)
     * @return corrected {@link ShotFrame}
     */
    public static ShotFrame compensate(ShotFrame raw, double distanceM) {
        double thetaModelRad = hoodToLaunchRad(raw);
        double v0 = raw.muzzleSpeed().baseUnitMagnitude();
        double vHoriz = v0 * Math.cos(thetaModelRad);
        double tFlight = (Math.abs(vHoriz) > 1e-6) ? (distanceM / vHoriz) : 1.0;

        return applyCorrection(
                raw,
                v0,
                tFlight,
                RobotStateRecorder.getPoseWorldRobotCurrent().getRotation().getY(),
                RobotStateRecorder.getPoseWorldRobotCurrent().getRotation().getX(),
                RobotStateRecorder.getPoseWorldRobotCurrent().getRotation().getZ(),
                readShooterHeightM());
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Internal helpers
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * Derives the actual shooter height from the world-Z of the shot frame.
     *
     * <p>When the robot is on flat ground, {@code getPoseWorldShotCurrent().getZ()} equals
     * {@code kRobotToShot.getZ()} (= 0.35 m). Any robot elevation adds directly to this value.
     * The model's nominal shooter height (0.63 m) is applied as an offset so that Δh = 0 on a
     * flat field.
     */
    private static double readShooterHeightM() {
        double actualShotZ = RobotStateRecorder.getPoseWorldShotCurrent().getZ();
        return NOMINAL_SHOOTER_HEIGHT_M + (actualShotZ - RobotStateRecorder.kRobotToShot.getZ());
    }

    /** Converts the hood angle in a {@link ShotFrame} back to launch angle (radians). */
    private static double hoodToLaunchRad(ShotFrame frame) {
        return Math.toRadians(90.0 - frame.hoodAngle().in(Degrees));
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Core solver
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * Applies all three corrections (height, elevation, roll-yaw) and returns the corrected frame.
     *
     * <h3>Elevation derivation</h3>
     *
     * <p>Launch direction in robot frame after turret yaw ψ_rel:
     * <pre>  v̂_robot = [cos(θ)·cos(ψ), cos(θ)·sin(ψ), sin(θ)]</pre>
     *
     * <p>World-frame Z component after robot pitch α (nose-up) and roll φ (right-down):
     * <pre>
     *   v_z / v₀ = A·sin(θ_cmd) + B·cos(θ_cmd)
     *   A = cos(α)·cos(φ)
     *   B = sin(α)·cos(ψ_rel) + cos(α)·sin(φ)·sin(ψ_rel)
     * </pre>
     *
     * <p>Height offset Δh lowers the required vertical velocity:
     * <pre>  C = sin(θ_model) − Δh / (v₀ · t_flight)</pre>
     *
     * <p>Closed-form: {@code θ_cmd = asin(C / √(A²+B²)) − atan2(B, A)}
     */
    private static ShotFrame applyCorrection(
            ShotFrame raw,
            double v0,
            double tFlight,
            double pitchRad,
            double rollRad,
            double robotYawRad,
            double shooterHeightM) {

        double turretYawWorldRad = raw.turretAngleWorld().in(Radians);
        double psiRel = turretYawWorldRad - robotYawRad;

        double thetaModelRad = hoodToLaunchRad(raw);
        double deltaH = shooterHeightM - NOMINAL_SHOOTER_HEIGHT_M;

        double thetaCmdRad =
                solveElevationRad(thetaModelRad, v0, tFlight, deltaH, pitchRad, rollRad, psiRel);
        double yawOffsetRad =
                solveLateralYawOffsetRad(thetaModelRad, pitchRad, rollRad, psiRel);

        log(pitchRad, rollRad, deltaH, psiRel, thetaModelRad, thetaCmdRad, yawOffsetRad);

        return new ShotFrame(
                raw.turretAngleWorld().plus(Radians.of(yawOffsetRad)),
                Degrees.of(90.0 - Math.toDegrees(thetaCmdRad)),
                raw.muzzleSpeed());
    }

    /**
     * Solves A·sin(θ) + B·cos(θ) = C for θ, clamped to the operating range.
     * Falls back to {@code thetaModelRad} if the system is infeasible.
     */
    private static double solveElevationRad(
            double thetaModelRad,
            double v0,
            double tFlight,
            double deltaH,
            double alpha,
            double phi,
            double psiRel) {

        double A = Math.cos(alpha) * Math.cos(phi);
        double B = Math.sin(alpha) * Math.cos(psiRel)
                + Math.cos(alpha) * Math.sin(phi) * Math.sin(psiRel);
        double C = Math.sin(thetaModelRad) - deltaH / (v0 * tFlight);
        double R = Math.hypot(A, B);

        if (R < 1e-9 || Math.abs(C / R) > 1.0) {
            return thetaModelRad;
        }

        double theta = Math.asin(C / R) - Math.atan2(B, A);
        return Math.max(Math.toRadians(THETA_MIN_DEG),
                Math.min(Math.toRadians(THETA_MAX_DEG), theta));
    }

    /**
     * Computes turret yaw offset to cancel the lateral component of the tilted launch direction.
     *
     * <p>World-frame gravity is purely vertical and does not contribute here. The drift comes
     * from Ry(−α)·Rx(φ) rotating the launch direction out of the intended vertical plane.
     * To first order in α and φ the lateral velocity perpendicular to the aim direction is:
     * <pre>
     *   v_lateral ≈ v₀ · sin(θ) · (α·sin(ψ_rel) − φ·cos(ψ_rel))
     * </pre>
     * Yaw correction (uses sin(φ)/sin(α) for accuracy beyond small angles):
     * <pre>
     *   δψ = asin( tan(θ_model) · (sin(φ)·cos(ψ_rel) − sin(α)·sin(ψ_rel)) )
     * </pre>
     */
    private static double solveLateralYawOffsetRad(
            double thetaModelRad, double alpha, double phi, double psiRel) {

        double tanTheta = Math.tan(thetaModelRad);
        double ratio = tanTheta
                * (Math.sin(phi) * Math.cos(psiRel) - Math.sin(alpha) * Math.sin(psiRel));
        return Math.asin(Math.max(-1.0, Math.min(1.0, ratio)));
    }

    private static void log(
            double pitchRad,
            double rollRad,
            double deltaH,
            double psiRelRad,
            double thetaModelRad,
            double thetaCmdRad,
            double yawOffsetRad) {
        Logger.recordOutput("ShotPoseCompensator/input/pitchDeg", Math.toDegrees(pitchRad));
        Logger.recordOutput("ShotPoseCompensator/input/rollDeg", Math.toDegrees(rollRad));
        Logger.recordOutput("ShotPoseCompensator/input/deltaH_m", deltaH);
        Logger.recordOutput("ShotPoseCompensator/input/psiRelDeg", Math.toDegrees(psiRelRad));
        Logger.recordOutput("ShotPoseCompensator/output/thetaModel_deg", Math.toDegrees(thetaModelRad));
        Logger.recordOutput("ShotPoseCompensator/output/thetaCmd_deg", Math.toDegrees(thetaCmdRad));
        Logger.recordOutput("ShotPoseCompensator/output/hoodAngleCorrected_deg",
                90.0 - Math.toDegrees(thetaCmdRad));
        Logger.recordOutput("ShotPoseCompensator/output/yawOffset_deg", Math.toDegrees(yawOffsetRad));
        Logger.recordOutput("ShotPoseCompensator/output/deltaTheta_deg",
                Math.toDegrees(thetaCmdRad - thetaModelRad));
    }
}
