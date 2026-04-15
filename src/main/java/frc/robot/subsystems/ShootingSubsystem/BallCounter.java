package frc.robot.subsystems.ShootingSubsystem;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotConstants;
import frc.robot.subsystems.ShootingSubsystem.ShotCalculator.TargetMode;
import frc.robot.utils.HubShiftUtil;
import frc.robot.utils.HubShiftUtil.ShiftEnum;
import lib.ironpulse.math.filter.EdgeFilter;
import lib.ironpulse.math.filter.LowPassFilter;
import lib.ironpulse.math.filter.MovingAverageFilter;
import org.littletonrobotics.junction.Logger;

/**
 * BallCounter tracks the number of balls shot during different game shifts. Uses edge detection on
 * shooter current to detect when a ball is shot.
 */
public class BallCounter {
    private static final int NUM_SHIFTS = 7;

    private final ShootingSuperstructure shootingSuperstructure;

    // Shot detection filters - same as old code
    private final LowPassFilter shooterVelocityLPF = new LowPassFilter(10, 0.0);
    private final EdgeFilter edgeFilter = new EdgeFilter(EdgeFilter.EdgeType.RISING, 20.0, 1.0, 1);

    // BPS calculation using time between shots
    private static final double BPS_TIMEOUT = 1.0; // Reset BPS to 0 after 1 second of no shots
    private double lastShotTime = 0.0;
    private double currentBPS = 0.0;
    private final MovingAverageFilter bpsFilter = new MovingAverageFilter(2, 0.0);

    // Overall counters
    private int ballCountAll = 0;
    private int ballCountGoal = 0;

    // Per-shift counters (AUTO, TRANSITION, SHIFT1-4, ENDGAME)
    private final int[] shiftCounters = new int[NUM_SHIFTS];

    public BallCounter(ShootingSuperstructure shootingSuperstructure) {
        this.shootingSuperstructure = shootingSuperstructure;
    }

    /** Updates the ball counter. Should be called periodically from computeRpm. */
    public void update(
            double shooterRpsCurr, double shooterRpsDes, Current shooterCurrent, TargetMode mode) {
        // Update filters - same as old code
        shooterVelocityLPF.input(shooterRpsCurr, RobotConstants.LOOPER_DT);
        edgeFilter.input(shooterCurrent.in(Amp), RobotConstants.LOOPER_DT);

        // Detect shot - EXACT same logic as old code
        boolean shot =
                edgeFilter.compute() == 1.0
                        && shootingSuperstructure.getIdx().getCurrentState()
                                == SpindexerSubsystem.State.FEED;

        // Update counters on shot
        if (shot) {
            ballCountAll++;
            if (mode == TargetMode.GOAL) {
                ballCountGoal++;
            }

            // Update shift-specific counter
            ShiftEnum currentShift = HubShiftUtil.getShiftedShiftInfo().currentShift();
            int shiftIndex = getShiftIndex(currentShift);
            if (shiftIndex >= 0) {
                shiftCounters[shiftIndex]++;
            }

            // Calculate BPS based on time between shots
            double currentTime = Timer.getFPGATimestamp();
            if (lastShotTime > 0) {
                double timeBetweenShots = currentTime - lastShotTime;
                if (timeBetweenShots > 0) {
                    currentBPS = 1.0 / timeBetweenShots;
                    bpsFilter.input(currentBPS, RobotConstants.LOOPER_DT);
                }
            }
            lastShotTime = currentTime;
        }

        // Reset BPS to 0 if no shots for a while
        double timeSinceLastShot = Timer.getFPGATimestamp() - lastShotTime;
        if (timeSinceLastShot > BPS_TIMEOUT) {
            currentBPS = 0.0;
            bpsFilter.input(0.0, RobotConstants.LOOPER_DT);
        }

        Logger.recordOutput("ShotCounter/BallCountAll", ballCountAll);
        Logger.recordOutput("ShotCounter/BallCountGoal", ballCountGoal);
        Logger.recordOutput("ShotCounter/AvgBPS", bpsFilter.compute());

        // Log per-shift counters (game sequence: AUTO, TRANSITION, SHIFT1-4, ENDGAME)
        Logger.recordOutput("ShotCounter/Auto", shiftCounters[0]);
        Logger.recordOutput("ShotCounter/Transition", shiftCounters[1]);
        Logger.recordOutput("ShotCounter/Shift1", shiftCounters[2]);
        Logger.recordOutput("ShotCounter/Shift2", shiftCounters[3]);
        Logger.recordOutput("ShotCounter/Shift3", shiftCounters[4]);
        Logger.recordOutput("ShotCounter/Shift4", shiftCounters[5]);
        Logger.recordOutput("ShotCounter/Endgame", shiftCounters[6]);
    }

    private int getShiftIndex(ShiftEnum shift) {
        return switch (shift) {
            case AUTO -> 0;
            case TRANSITION -> 1;
            case SHIFT1 -> 2;
            case SHIFT2 -> 3;
            case SHIFT3 -> 4;
            case SHIFT4 -> 5;
            case ENDGAME -> 6;
            default -> -1; // DISABLED - not tracked
        };
    }

    /** Resets all ball counters (overall and per-shift). */
    public void resetAll() {
        ballCountAll = 0;
        ballCountGoal = 0;
        for (int i = 0; i < NUM_SHIFTS; i++) {
            shiftCounters[i] = 0;
        }
    }

    /**
     * Resets the counter for a specific shift.
     *
     * @param shiftId Shift ID (0=AUTO, 1=TRANSITION, 2=SHIFT1, 3=SHIFT2, 4=SHIFT3, 5=SHIFT4,
     *     6=ENDGAME)
     */
    public void resetShift(int shiftId) {
        if (shiftId >= 0 && shiftId < NUM_SHIFTS) {
            shiftCounters[shiftId] = 0;
        }
    }

    public int getBallCountAll() {
        return ballCountAll;
    }

    public int getBallCountGoal() {
        return ballCountGoal;
    }

    /**
     * Gets the ball count for a specific shift.
     *
     * @param shiftId Shift ID (0=AUTO, 1=TRANSITION, 2=SHIFT1, 3=SHIFT2, 4=SHIFT3, 5=SHIFT4,
     *     6=ENDGAME)
     * @return Ball count for that shift, or 0 if invalid ID
     */
    public int getShiftCount(int shiftId) {
        if (shiftId >= 0 && shiftId < NUM_SHIFTS) {
            return shiftCounters[shiftId];
        }
        return 0;
    }

    public double getBallsPerSecond() {
        return bpsFilter.compute();
    }
}
