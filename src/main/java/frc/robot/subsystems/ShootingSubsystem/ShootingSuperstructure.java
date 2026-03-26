package frc.robot.subsystems.ShootingSubsystem;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants;
import frc.robot.RobotConstants;
import frc.robot.RobotStateRecorder;
import frc.robot.subsystems.Configs.ShooterParamsNT;
import frc.robot.subsystems.Configs.ShotCalculatorParamsNT;
import frc.robot.subsystems.Configs.SpindexerParamsNT;
import frc.robot.subsystems.ShootingSubsystem.ShotCalculator.TargetMode;
import frc.robot.subsystems.ShootingSubsystem.TurretSubsystem.TurretMode;
import java.util.function.Supplier;
import lib.ironpulse.io.MotorIO;
import lib.ironpulse.io.MotorInputsAutoLogged;
import lib.ironpulse.math.filter.EdgeFilter;
import lib.ironpulse.math.filter.LowPassFilter;
import lib.ironpulse.math.filter.MovingAverageFilter;
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
    private final LowPassFilter shooterVelocityLPF = new LowPassFilter(10, 0.0);
    private final EdgeFilter edgeFilter = new EdgeFilter(EdgeFilter.EdgeType.RISING, 20.0, 1.0, 1);
    private int ballCounter = 0;
    private int ballCounterShoot = 0;
    private TargetMode mode;
    @Getter private boolean isShooting = false;
    private MovingAverageFilter BPSCalculator = new MovingAverageFilter(2, 0);

    public ShootingSuperstructure(
            TurretSubsystem turret,
            PositionMotorSubsystem<MotorInputsAutoLogged, MotorIO, Angle> hood,
            VelocityMotorSubsystem<MotorInputsAutoLogged, MotorIO> shooter,
            SpindexerSubsystem idx) {
        this.turret = turret;
        this.hood = hood;
        this.shooter = shooter;
        this.idx = idx;
    }

    public ShotFrame getCurrentFrame() {
        Angle turretWorldRotation =
                RobotStateRecorder.getPoseWorldShotCurrent().toPose2d().getRotation().getMeasure();
        Angle bba = hood.getCurrPos();
        double rpmA = ShotCalculatorParamsNT.rpmA.getValue();
        AngularVelocity shooterVel = shooter.getVelocity();

        double rpm = shooterVel.in(RotationsPerSecond) * 60.0;
        double muzzleSpeedMps =
                (rpm
                                - ShotCalculatorParamsNT.rpmB.getValue() * bba.in(Degrees)
                                - ShotCalculatorParamsNT.rpmC.getValue())
                        / rpmA;
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
                                                51))));
    }

    public Command shootWhenReady(boolean forceFeed) {
        return Commands.parallel(
                runFrame(),
                Commands.waitUntil(shooter::velocityAtGoal)
                        .andThen(
                                Commands.runOnce(() -> isShooting = true),
                                idx.runState(
                                        () ->
                                                turret.getCurrentMode() == TurretMode.TRACKING
                                                        ? forceFeed
                                                                ? IdxMode.FORCE_FEED
                                                                : IdxMode.FEED
                                                        : IdxMode.OFF))
                        .finallyDo(() -> isShooting = false));
    }

    public Command shootOnCondition(Supplier<Boolean> conditional) {
        return Commands.parallel(
                runFrame(),
                Commands.waitUntil(shooter::velocityAtGoal)
                        .andThen(
                                Commands.runOnce(() -> isShooting = true),
                                idx.runState(
                                        () ->
                                                conditional.get().booleanValue()
                                                        ? IdxMode.FEED
                                                        : IdxMode.OFF))
                        .finallyDo(() -> isShooting = false));
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

        double baseRpm =
                ShotCalculatorParamsNT.rpmA.getValue() * muzzleSpeed.in(MetersPerSecond)
                        + ShotCalculatorParamsNT.rpmB.getValue() * bba.in(Degrees)
                        + ShotCalculatorParamsNT.rpmC.getValue();
        double scaledRpm = baseRpm * scalingFactor;

        Logger.recordOutput("ShootingSuperstructure/Distance/currentMeters", currentDistance);
        Logger.recordOutput("ShootingSuperstructure/Distance/baseRpm", baseRpm);
        Logger.recordOutput("ShootingSuperstructure/Distance/scaledRpm", scaledRpm);
        SmartDashboard.putNumber("ShootingSuperstructure/Distance/currentMeters", currentDistance);
        SmartDashboard.putNumber("ShootingSuperstructure/Distance/baseRpm", baseRpm);
        SmartDashboard.putNumber("ShootingSuperstructure/Distance/scaledRpm", scaledRpm);

        double shooterRpsCurr = shooter.getVelocity().in(RotationsPerSecond);
        double shooterRpsDes = shooter.getCurrSetpoint().in(RotationsPerSecond);
        double shooterCurrent = shooter.getSupplyCurrent().in(Amp);
        shooterVelocityLPF.input(shooterRpsCurr, RobotConstants.LOOPER_DT);
        edgeFilter.input(shooterCurrent, RobotConstants.LOOPER_DT);
        boolean shot =
                edgeFilter.compute() == 1.0
                        && shooter.velocityAtGoal(RotationsPerSecond.of(5.0))
                        && idx.getCurrentState() == SpindexerSubsystem.State.FEED;
        ballCounter += shot ? 1 : 0;
        ballCounterShoot += (shot && mode == TargetMode.GOAL) ? 1 : 0;
        BPSCalculator.input(shot ? 1.0 : 0.0, RobotConstants.LOOPER_DT);

        Logger.recordOutput("ShotCounter/RpsCurr", shooterRpsCurr);
        Logger.recordOutput("ShotCounter/RpsDes", shooterRpsDes);
        Logger.recordOutput("ShotCounter/Current", shooterCurrent);
        Logger.recordOutput("ShotCounter/BallDetected", shot);
        Logger.recordOutput("ShotCounter/BallCountAll", ballCounter);
        Logger.recordOutput("ShotCounter/BallCountShoot", ballCounterShoot);
        Logger.recordOutput(
                "ShotCounter/AvgBPS", BPSCalculator.compute() * BPSCalculator.getWindowSize());

        return scaledRpm;
    }

    @AutoLogOutput(key = "ShootingSuperstructure/readyToShoot")
    public boolean readyToShoot() {
        return turret.atGoal() && hood.positionAtGoal() && shooter.velocityAtGoal();
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
                            ShotFrame frame = RobotStateRecorder.getCmdFrame();
                            Angle bba = Degrees.of(77);
                            double rpm = computeRpm(MetersPerSecond.of(6), bba);
                            return RotationsPerSecond.of(rpm / 60.0);
                        }));
    }

    public Command runResetBallCounter() {
        return Commands.runOnce(
                () -> {
                    ballCounter = 0;
                });
    }

    public enum IdxMode {
        OFF,
        FEED,
        FORCE_FEED,
        REVERSE
    }
}
