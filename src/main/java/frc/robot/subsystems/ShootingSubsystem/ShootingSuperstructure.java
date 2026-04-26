package frc.robot.subsystems.ShootingSubsystem;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants;
import frc.robot.RobotStateRecorder;
import frc.robot.subsystems.Configs.ShooterParamsNT;
import frc.robot.subsystems.Configs.ShotCalculatorParamsFEEDNT;
import frc.robot.subsystems.Configs.ShotCalculatorParamsGOALNT;
import frc.robot.subsystems.Configs.ShotCalculatorParamsNT;
import frc.robot.subsystems.Configs.SpindexerParamsNT;
import frc.robot.subsystems.ShootingSubsystem.ShotCalculator.TargetMode;
import frc.robot.subsystems.ShootingSubsystem.TurretSubsystem.TurretMode;
import frc.robot.utils.HubShiftUtil;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import lib.ironpulse.command.RumbleWhenCommand;
import lib.ironpulse.command.VisualizeProjectileShot;
import lib.ironpulse.io.MotorIO;
import lib.ironpulse.io.MotorInputsAutoLogged;
import lib.ironpulse.subsystem.position.PositionMotorSubsystem;
import lib.ironpulse.subsystem.velocity.VelocityMotorSubsystem;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ShootingSuperstructure {
    private final TurretSubsystem turret;
    private final PositionMotorSubsystem<MotorInputsAutoLogged, MotorIO, Angle> hood;
    @Getter private final VelocityMotorSubsystem<MotorInputsAutoLogged, MotorIO> shooter;
    @Getter private final SpindexerSubsystem idx;
    @Getter private final BallCounter ballCounter;
    @Getter public TargetMode mode = TargetMode.GOAL;
    @Getter private boolean isShooting = false;
    @Getter public boolean isGoalShooting = false;

    // Ball tracking for visualization
    private static class ProjectileBall {
        final Pose3d launchPose;
        final Rotation2d launchYaw;
        final Rotation2d launchPitch;
        final double launchSpeed;
        final Translation2d launchVelocity;
        final double launchTime;

        ProjectileBall(
                Pose3d pose,
                Rotation2d yaw,
                Rotation2d pitch,
                double speed,
                Translation2d vel,
                double time) {
            this.launchPose = pose;
            this.launchYaw = yaw;
            this.launchPitch = pitch;
            this.launchSpeed = speed;
            this.launchVelocity = vel;
            this.launchTime = time;
        }
    }

    private final List<ProjectileBall> activeBalls = new ArrayList<>();
    private double lastBallLaunchTime = 0.0;
    private static final double BALL_LAUNCH_INTERVAL = 0.1; // 0.1 seconds between balls
    private final ShotCalculator calculator = new ShotCalculator();

    public ShootingSuperstructure(
            TurretSubsystem turret,
            PositionMotorSubsystem<MotorInputsAutoLogged, MotorIO, Angle> hood,
            VelocityMotorSubsystem<MotorInputsAutoLogged, MotorIO> shooter,
            SpindexerSubsystem idx) {
        this.turret = turret;
        this.hood = hood;
        this.shooter = shooter;
        this.idx = idx;
        this.ballCounter = new BallCounter(this);
    }

    public ShotFrame getCurrentFrame() {
        Angle turretWorldRotation =
                RobotStateRecorder.getPoseWorldShotCurrent().toPose2d().getRotation().getMeasure();
        Angle bba = hood.getCurrPos();
        double rpmA =
                calculator.decideShotMode() == TargetMode.GOAL
                        ? ShotCalculatorParamsGOALNT.rpmA.getValue()
                        : ShotCalculatorParamsFEEDNT.rpmA.getValue();
        double rpmB =
                calculator.decideShotMode() == TargetMode.GOAL
                        ? ShotCalculatorParamsGOALNT.rpmB.getValue()
                        : ShotCalculatorParamsFEEDNT.rpmB.getValue();
        double rpmC =
                calculator.decideShotMode() == TargetMode.GOAL
                        ? ShotCalculatorParamsGOALNT.rpmC.getValue()
                        : ShotCalculatorParamsFEEDNT.rpmC.getValue();

        AngularVelocity shooterVel = shooter.getVelocity();
        double rpm = shooterVel.in(RotationsPerSecond) * 60.0;
        double muzzleSpeedMps = (rpm - rpmB * bba.in(Degrees) - rpmC) / rpmA;
        return new ShotFrame(turretWorldRotation, bba, MetersPerSecond.of(muzzleSpeedMps));
    }

    public void setDefaultCommand() {
        turret.setDefaultCommand(turret.runTurretTargetLoop());
        idx.setDefaultCommand(idx.runState(() -> IdxMode.OFF));
        shooter.setDefaultCommand(
                shooter.runVelVolt(
                        () -> RotationsPerSecond.of(ShooterParamsNT.idleVelRPS.getValue())));
        hood.setDefaultCommand(
                hood.runPosition(
                        () ->
                                Degrees.of(
                                        MathUtil.clamp(
                                                RobotStateRecorder.getCmdFrame()
                                                        .hoodAngle()
                                                        .in(Degrees),
                                                0,
                                                47))));
    }

    public Command shootWhenReady(boolean forceFeed) {
        return Commands.parallel(
                runFrame(),
                new VisualizeProjectileShot(
                                () -> RobotStateRecorder.getPoseWorldShotCurrent(),
                                () ->
                                        Rotation2d.fromRadians(
                                                RobotStateRecorder.getCmdFrame()
                                                        .turretAngleWorld()
                                                        .in(Radians)),
                                () ->
                                        Rotation2d.fromRadians(
                                                RobotStateRecorder.getCmdFrame()
                                                        .hoodAngle()
                                                        .in(Radians)),
                                () ->
                                        RobotStateRecorder.getCmdFrame()
                                                .muzzleSpeed()
                                                .in(MetersPerSecond),
                                () ->
                                        RobotStateRecorder.getVelocityWorldRobotCurrent()
                                                .getTranslation(),
                                true)
                        .onlyIf(() -> RobotBase.isSimulation()),
                Commands.waitUntil(shooter::velocityAtGoal)
                        .andThen(
                                Commands.runOnce(() -> isShooting = true),
                                Commands.runOnce(() -> isGoalShooting = true)
                                        .onlyIf(
                                                () ->
                                                        calculator.decideShotMode()
                                                                == TargetMode.GOAL),
                                idx.runState(
                                        () ->
                                                turret.getCurrentMode() == TurretMode.TRACKING
                                                                && !isInTower()
                                                                && !isInHubZone()
                                                        ? forceFeed
                                                                ? IdxMode.FORCE_FEED
                                                                : IdxMode.FEED
                                                        : IdxMode.OFF))
                        .finallyDo(
                                () -> {
                                    isShooting = false;
                                    isGoalShooting = false;
                                }));
    }

    public Command shootWhenReady(boolean forceFeed, GenericHID... hids) {
        return Commands.parallel(
                runFrame(),
                new VisualizeProjectileShot(
                                () -> RobotStateRecorder.getPoseWorldShotCurrent(),
                                () ->
                                        Rotation2d.fromRadians(
                                                RobotStateRecorder.getCmdFrame()
                                                        .turretAngleWorld()
                                                        .in(Radians)),
                                () ->
                                        Rotation2d.fromRadians(
                                                RobotStateRecorder.getCmdFrame()
                                                        .hoodAngle()
                                                        .in(Radians)),
                                () ->
                                        RobotStateRecorder.getCmdFrame()
                                                .muzzleSpeed()
                                                .in(MetersPerSecond),
                                () ->
                                        RobotStateRecorder.getVelocityWorldRobotCurrent()
                                                .getTranslation(),
                                true)
                        .onlyIf(() -> RobotBase.isSimulation()),
                new RumbleWhenCommand(
                        () ->
                                (mode == TargetMode.GOAL
                                        && !HubShiftUtil.getShiftedShiftInfo().active()),
                        hids),
                Commands.waitUntil(shooter::velocityAtGoal)
                        .andThen(
                                Commands.runOnce(() -> isShooting = true),
                                Commands.runOnce(() -> isGoalShooting = true)
                                        .onlyIf(
                                                () ->
                                                        calculator.decideShotMode()
                                                                == TargetMode.GOAL),
                                idx.runState(
                                        () ->
                                                turret.getCurrentMode() == TurretMode.TRACKING
                                                                && !isInTower()
                                                                && !isInHubZone()
                                                        ? forceFeed
                                                                ? IdxMode.FORCE_FEED
                                                                : IdxMode.FEED
                                                        : IdxMode.OFF))
                        .finallyDo(
                                () -> {
                                    isShooting = false;
                                    isGoalShooting = false;
                                }));
    }

    public Command shootOnCondition(Supplier<Boolean> conditional) {
        return Commands.parallel(
                runFrame(),
                Commands.waitUntil(shooter::velocityAtGoal)
                        .andThen(
                                idx.runState(() -> IdxMode.REVERSE)
                                        .withTimeout(SpindexerParamsNT.unjammTimeoutSec.getValue()),
                                Commands.runOnce(() -> isShooting = true),
                                idx.runState(
                                        () ->
                                                conditional.get()
                                                                && turret.getCurrentMode()
                                                                        == TurretMode.TRACKING
                                                                && !isInTower()
                                                                && !isInHubZone()
                                                        ? IdxMode.FEED
                                                        : IdxMode.OFF))
                        .finallyDo(
                                () -> {
                                    isShooting = false;
                                    isGoalShooting = false;
                                }));
    }

    public Command runFrame() {
        return Commands.parallel(
                turret.setTurretPoseWorld(
                        () -> RobotStateRecorder.getCmdFrame().turretAngleWorld()),
                hood.runPosition(
                        () ->
                                Degrees.of(
                                        MathUtil.clamp(
                                                RobotStateRecorder.getCmdFrame()
                                                        .hoodAngle()
                                                        .in(Degrees),
                                                0,
                                                51))),
                shooter.runVelVolt(
                        () -> {
                            ShotFrame frame = RobotStateRecorder.getCmdFrame();
                            Angle bba = frame.hoodAngle();
                            double rpm = computeRpm(frame.muzzleSpeed(), bba);
                            return RotationsPerSecond.of(rpm / 60.0);
                        }));
    }

    public Command runInitialFrame() {
        return Commands.parallel(
                turret.setTurretPoseWorld(() -> calculator.computeGoalFrame().turretAngleWorld()),
                shooter.runVelVolt(
                        () -> {
                            ShotFrame frame = calculator.computeGoalFrame();
                            Angle bba = frame.hoodAngle();
                            double rpm = computeRpm(frame.muzzleSpeed(), bba);
                            return RotationsPerSecond.of(rpm / 60.0);
                        }));
    }

    public Command runFrame(Supplier<IdxMode> idxModeSupplier) {
        return Commands.parallel(runFrame(), idx.runState(idxModeSupplier));
    }

    private double computeRpm(LinearVelocity muzzleSpeed, Angle bba) {
        double currentDistance = getDistance();
        double scalingFactor = 1.0;
        if (currentDistance > 2.5) {
            double minDistance = 2.5;
            double maxDistance = 8;
            if (currentDistance >= maxDistance) {
                scalingFactor = ShotCalculatorParamsNT.distanceScaler.getValue();
            } else {
                double t_distance = (currentDistance - minDistance) / (maxDistance - minDistance);
                scalingFactor =
                        1.0 + (ShotCalculatorParamsNT.distanceScaler.getValue() - 1.0) * t_distance;
            }
        }

        // Use mode-specific parameters - use the mode already decided by calculator
        double rpmA =
                calculator.decideShotMode() == TargetMode.GOAL
                        ? ShotCalculatorParamsGOALNT.rpmA.getValue()
                        : ShotCalculatorParamsFEEDNT.rpmA.getValue();
        double rpmB =
                calculator.decideShotMode() == TargetMode.GOAL
                        ? ShotCalculatorParamsGOALNT.rpmB.getValue()
                        : ShotCalculatorParamsFEEDNT.rpmB.getValue();
        double rpmC =
                calculator.decideShotMode() == TargetMode.GOAL
                        ? ShotCalculatorParamsGOALNT.rpmC.getValue()
                        : ShotCalculatorParamsFEEDNT.rpmC.getValue();

        double baseRpm = rpmA * muzzleSpeed.in(MetersPerSecond) + rpmB * bba.in(Degrees) + rpmC;
        double scaledRpm = baseRpm * scalingFactor;

        Logger.recordOutput("ShootingSuperstructure/Distance/currentMeters", currentDistance);
        Logger.recordOutput("ShootingSuperstructure/Distance/baseRpm", baseRpm);
        Logger.recordOutput("ShootingSuperstructure/Distance/scaledRpm", scaledRpm);
        SmartDashboard.putNumber("ShootingSuperstructure/Distance/currentMeters", currentDistance);
        SmartDashboard.putNumber("ShootingSuperstructure/Distance/baseRpm", baseRpm);
        SmartDashboard.putNumber("ShootingSuperstructure/Distance/scaledRpm", scaledRpm);

        double shooterRpsCurr = shooter.getVelocity().in(RotationsPerSecond);
        double shooterRpsDes = shooter.getCurrSetpoint().in(RotationsPerSecond);

        // Update ball counter with current shooter state
        ballCounter.update(shooterRpsCurr, shooterRpsDes, shooter.getSupplyCurrent(), mode);

        return scaledRpm;
    }

    @AutoLogOutput(key = "ShootingSuperstructure/readyToShoot")
    public boolean readyToShoot() {
        return turret.atGoal() && hood.positionAtGoal() && shooter.velocityAtGoal();
    }

    @AutoLogOutput(key = "ShootingSuperstructure/isInTower")
    public boolean isInTower() {
        Translation2d pos =
                RobotStateRecorder.getPoseWorldRobotCurrent().getTranslation().toTranslation2d();
        double x = pos.getX(), y = pos.getY();
        boolean inBlue =
                x <= FieldConstants.Tower.frontFaceX
                        && Math.abs(y - FieldConstants.Tower.centerPoint.getY())
                                <= FieldConstants.Tower.width / 2.0;
        boolean inRed =
                x >= FieldConstants.fieldLength - FieldConstants.Tower.frontFaceX
                        && Math.abs(y - FieldConstants.Tower.oppCenterPoint.getY())
                                <= FieldConstants.Tower.width / 2.0;
        return inBlue || inRed;
    }

    @AutoLogOutput(key = "ShootingSuperstructure/isInHubZone")
    public boolean isInHubZone() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        boolean isBlue = alliance.get() == Alliance.Blue;

        Translation2d pos =
                RobotStateRecorder.getPoseWorldRobotCurrent().getTranslation().toTranslation2d();
        double x = pos.getX(), y = pos.getY();
        boolean inBlue =
                x >= 5
                        && x <= 7.3
                        && Math.abs(y - FieldConstants.Hub.topCenterPoint.getX())
                                <= FieldConstants.Hub.width / 2.0;
        boolean inRed =
                x <= FieldConstants.fieldLength - 5
                        && x >= FieldConstants.fieldLength - 7.3
                        && Math.abs(y - FieldConstants.Hub.oppTopCenterPoint.getX())
                                <= FieldConstants.Hub.width / 2.0;
        return isBlue ? inBlue : inRed;
    }

    public Command runUnjamming() {
        return idx.runState(() -> IdxMode.REVERSE)
                .withTimeout(Seconds.of(SpindexerParamsNT.unjammTimeoutSec.getValue()));
    }

    public Command runForceFeeding() {
        return idx.runState(() -> IdxMode.FORCE_FEED);
    }

    public Command runZero() {
        return hood.zeroCommand();
    }

    public double getDistance() {
        double xAlliance = RobotStateRecorder.getPoseDriverRobotCurrent().getX();
        if (xAlliance <= FieldConstants.LinesVertical.allianceZone) {
            mode = TargetMode.GOAL;
        } else {
            mode = TargetMode.FEED;
        }

        if (mode == TargetMode.GOAL) {
            return RobotStateRecorder.getTranslationShotToTargetCurrent(
                            RobotStateRecorder.kFrameGoal)
                    .getNorm();
        } else {
            double yAlliance = RobotStateRecorder.getPoseDriverRobotCurrent().getY();
            String feedFrame =
                    yAlliance > FieldConstants.fieldWidth / 2.0
                            ? RobotStateRecorder.kFrameFeedUp
                            : RobotStateRecorder.kFrameFeedDown;
            return RobotStateRecorder.getTranslationShotToTargetCurrent(feedFrame).getNorm();
        }
    }

    public Command runSetFrame() {
        return Commands.parallel(
                turret.runTurretToZero(),
                hood.runPosition(() -> Degrees.of(19)),
                idx.runState(() -> IdxMode.FEED),
                shooter.runVelVolt(
                        () -> {
                            Angle bba = Degrees.of(77);
                            double rpm = computeRpm(MetersPerSecond.of(6), bba);
                            return RotationsPerSecond.of(rpm / 60);
                        }));
    }

    /**
     * Visualizes projectile balls shot at 0.1s intervals with their current positions based on
     * ballistic trajectory simulation.
     */
    public void visualizeProjectileBalls(double currentTimeSec) {
        // Add new ball if shooting and enough time has passed since last ball
        if (isShooting
                && (currentTimeSec - lastBallLaunchTime) >= BALL_LAUNCH_INTERVAL
                && RobotBase.isSimulation()) {
            // Get current shot parameters at launch
            Pose3d releasePose = RobotStateRecorder.getPoseWorldShotCurrent();
            Rotation2d yawWorld =
                    Rotation2d.fromRadians(
                            RobotStateRecorder.getCmdFrame().turretAngleWorld().in(Radians));
            Rotation2d pitch =
                    Rotation2d.fromRadians(
                            RobotStateRecorder.getCmdFrame().hoodAngle().in(Radians));
            double muzzleSpeed = RobotStateRecorder.getCmdFrame().muzzleSpeed().in(MetersPerSecond);
            Translation2d addedVelocity =
                    RobotStateRecorder.getVelocityWorldRobotCurrent().getTranslation();

            activeBalls.add(
                    new ProjectileBall(
                            releasePose,
                            yawWorld,
                            pitch,
                            muzzleSpeed,
                            addedVelocity,
                            currentTimeSec));
            lastBallLaunchTime = currentTimeSec;
        }

        // Update all active balls and calculate their current positions
        List<Pose3d> ballPoses = new ArrayList<>();
        activeBalls.removeIf(
                ball -> {
                    double timeSinceLaunch = currentTimeSec - ball.launchTime;

                    // Calculate current position of this ball
                    Pose3d ballPose =
                            calculateProjectilePosition(
                                    ball.launchPose,
                                    ball.launchYaw,
                                    ball.launchPitch,
                                    ball.launchSpeed,
                                    ball.launchVelocity,
                                    timeSinceLaunch);

                    // Remove ball if it hit the ground or is too old
                    if (ballPose == null || ballPose.getZ() <= 0.0 || timeSinceLaunch > 3.0) {
                        return true; // Remove this ball
                    }

                    ballPoses.add(ballPose);
                    return false; // Keep this ball
                });

        // Log all ball poses
        Logger.recordOutput("RobotComponents/ProjectileBalls", ballPoses.toArray(new Pose3d[0]));
    }

    /**
     * Calculate the position of a projectile at a given time after release using gravity-only
     * ballistic simulation.
     */
    private Pose3d calculateProjectilePosition(
            Pose3d releasePose,
            Rotation2d yawWorld,
            Rotation2d pitch,
            double muzzleSpeedMps,
            Translation2d addedVelocityMps,
            double timeSec) {
        if (timeSec < 0) return null;

        // Calculate initial velocity
        double yawRad = yawWorld.getRadians();
        double pitchRad = pitch.getRadians();
        double cosPitch = Math.cos(pitchRad);

        Translation3d vProjectile =
                new Translation3d(
                        muzzleSpeedMps * cosPitch * Math.cos(yawRad),
                        muzzleSpeedMps * cosPitch * Math.sin(yawRad),
                        muzzleSpeedMps * Math.sin(pitchRad));

        Translation3d vAdd3 =
                new Translation3d(addedVelocityMps.getX(), addedVelocityMps.getY(), 0.0);
        Translation3d initialVel = vProjectile.plus(vAdd3);

        // Apply gravity (9.81 m/s^2 downward)
        double gravity = 9.81;
        Translation3d pos = releasePose.getTranslation();

        // Position after time t: p = p0 + v0*t + 0.5*a*t^2
        double x = pos.getX() + initialVel.getX() * timeSec;
        double y = pos.getY() + initialVel.getY() * timeSec;
        double z = pos.getZ() + initialVel.getZ() * timeSec - 0.5 * gravity * timeSec * timeSec;

        if (z < 0.0) return null; // Ball hit ground

        return new Pose3d(new Translation3d(x, y, z), new Rotation3d());
    }

    /**
     * Visualizes robot components (turret, intake, spindexer) as Pose3d for AdvantageScope. Call
     * this method periodically to update the visualization.
     */
    public void visualizeComponents(double intakeExtensionMeters, double spindexerRotationRadians) {
        // Turret visualization: only yaw rotation from robot-relative angle, all else zero
        Angle turretRobotAngle = turret.getPosition();
        Pose3d turretPose =
                new Pose3d(
                        new Translation3d(0, 0, 0),
                        new Rotation3d(0, 0, turretRobotAngle.in(Radians)));
        Logger.recordOutput("RobotComponents/Turret", turretPose);

        // Intake visualization: linear interpolation from retracted to extended
        // Retracted: (-0.23732, 0, 0.03), Extended: (0, 0, 0) - swapped x and y
        // Assuming intakeExtensionMeters ranges from 0 (retracted) to max (extended)
        // We need to know the max extension value to interpolate correctly
        // For now, let's assume the extension value is normalized or we interpolate based on
        // position
        double maxExtension = 0.305; // Adjust based on your robot's max extension
        double t = MathUtil.clamp(intakeExtensionMeters / maxExtension, 0.0, 1.0);

        double retractedX = -0.23732;
        double retractedZ = 0.03;
        double extendedX = 0.0;
        double extendedZ = 0.0;

        double currentX = retractedX + (extendedX - retractedX) * t;
        double currentZ = retractedZ + (extendedZ - retractedZ) * t;

        Pose3d intakePose =
                new Pose3d(new Translation3d(currentX, 0, currentZ), new Rotation3d(0, 0, 0));
        Logger.recordOutput("RobotComponents/Intake", intakePose);

        // Spindexer visualization: rotating yaw based on spindexer rotation (negated)
        Pose3d spindexerPose =
                new Pose3d(
                        new Translation3d(0, 0, 0),
                        new Rotation3d(0, 0, -spindexerRotationRadians));
        Logger.recordOutput("RobotComponents/Spindexer", spindexerPose);
    }

    public Command runResetBallCounter() {
        return Commands.run(() -> ballCounter.resetAll());
    }

    public enum IdxMode {
        OFF,
        FEED,
        FORCE_FEED,
        REVERSE
    }
}
