package frc.robot.subsystems.ShootingSubsystem;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants;
import frc.robot.RobotStateRecorder;
import frc.robot.subsystems.Configs.ShooterParamsNT;
import frc.robot.subsystems.Configs.ShotCalculatorParamsNT;
import frc.robot.subsystems.Configs.SpindexerParamsNT;
import frc.robot.subsystems.ShootingSubsystem.ShotCalculator.TargetMode;
import frc.robot.subsystems.ShootingSubsystem.TurretSubsystem.TurretMode;
import java.util.function.Supplier;
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
    private final VelocityMotorSubsystem<MotorInputsAutoLogged, MotorIO> shooter;
    private final SpindexerSubsystem idx;
    @Getter private boolean isShooting = false;

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
        double hoodB = ShotCalculatorParamsNT.hoodB.getValue();
        double rpmA = ShotCalculatorParamsNT.rpmA.getValue();
        Angle modelHood = bba.minus(Degrees.of(ShotCalculatorParamsNT.hoodC.getValue())).div(hoodB);
        AngularVelocity shooterVel = shooter.getVelocity();

        double rpm = shooterVel.in(RotationsPerSecond) * 60.0;
        double muzzleSpeedMps =
                (rpm
                                - ShotCalculatorParamsNT.rpmB.getValue() * bba.in(Degrees)
                                - ShotCalculatorParamsNT.rpmC.getValue())
                        / rpmA;
        return new ShotFrame(turretWorldRotation, modelHood, MetersPerSecond.of(muzzleSpeedMps));
    }

    public void setDefaultCommand() {
        turret.setDefaultCommand(turret.runTurretTargetLoop());
        idx.setDefaultCommand(idx.runState(() -> IdxMode.OFF));
        shooter.setDefaultCommand(
                shooter.runVelVolt(
                        () -> RotationsPerSecond.of(ShooterParamsNT.idleVelRPS.getValue())));
        hood.setDefaultCommand(
                hood.runPosition(() -> computeBBA(RobotStateRecorder.getCmdFrame().hoodAngle())));
    }

    public Command shootWhenReady(boolean forceFeed) {
        return Commands.parallel(
                runFrame(),
                Commands.waitUntil(() -> shooter.velocityAtGoal())
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

    public Command runFrame() {
        return Commands.parallel(
                turret.setTurretPoseWorld(
                        () -> RobotStateRecorder.getCmdFrame().turretAngleWorld()),
                hood.runPosition(() -> computeBBA(RobotStateRecorder.getCmdFrame().hoodAngle())),
                shooter.runVelVolt(
                        () -> {
                            ShotFrame frame = RobotStateRecorder.getCmdFrame();
                            Angle bba = computeBBA(frame.hoodAngle());
                            double rpm = computeRpm(frame.muzzleSpeed(), bba);
                            return RotationsPerSecond.of(rpm / 60.0);
                        }));
    }

    public Command runFrame(Supplier<IdxMode> idxModeSupplier) {
        return Commands.parallel(runFrame(), idx.runState(idxModeSupplier));
    }

    private Angle computeBBA(Angle modelAngle) {
        return modelAngle
                .times(ShotCalculatorParamsNT.hoodB.getValue())
                .plus(Degrees.of(ShotCalculatorParamsNT.hoodC.getValue()));
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

        return (ShotCalculatorParamsNT.rpmA.getValue() * muzzleSpeed.in(MetersPerSecond)
                        + ShotCalculatorParamsNT.rpmB.getValue() * bba.in(Degrees)
                        + ShotCalculatorParamsNT.rpmC.getValue())
                * scalingFactor;
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

    public double getDistance() {
        TargetMode mode;
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

    public enum IdxMode {
        OFF,
        FEED,
        FORCE_FEED,
        REVERSE
    }
}
