package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.RobotStateRecorder;
import frc.robot.subsystems.Configs.ShooterParamsNT;
import lib.ironpulse.io.MotorIO;
import lib.ironpulse.io.MotorInputsAutoLogged;
import lib.ironpulse.subsystem.position.PositionMotorSubsystem;
import lib.ironpulse.subsystem.velocity.VelocityMotorSubsystem;
import lombok.Getter;

public class ShootingSuperstructure {

    private final TurretSubsystem turret;
    private final PositionMotorSubsystem<MotorInputsAutoLogged, MotorIO, Angle> hood;
    @Getter private final VelocityMotorSubsystem<MotorInputsAutoLogged, MotorIO> shooter;
    @Getter private final SpindexerSubsystem idx;

    public enum IdxMode {
        OFF,
        FEED,
        FORCE_FEED,
        REVERSE
    }

    public ShootingSuperstructure(
            VelocityMotorSubsystem<MotorInputsAutoLogged, MotorIO> shooter,
            PositionMotorSubsystem<MotorInputsAutoLogged, MotorIO, Angle> hood,
            TurretSubsystem turret,
            SpindexerSubsystem idx) {
        this.shooter = shooter;
        this.turret = turret;
        this.idx = idx;
        this.hood = hood;
    }

    public void setDefaultCommand() {
        turret.setDefaultCommand(turret.runTurretTargetLoop());
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

    public Command runShoot() {
        return Commands.parallel(
                turret.setTurretPoseWorld(
                        () -> RobotStateRecorder.getCmdFrame().turretAngleWorld()),
                hood.runPosition(() -> Degrees.of(45)),
                shooter.runVelVolt(
                        () -> RotationsPerSecond.of(ShooterParamsNT.runVelRPS.getValue())));
    }
}
