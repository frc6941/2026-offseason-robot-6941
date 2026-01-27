package frc.robot.subsystems.ShootingSubsystem;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Configs.ShooterParamsNT;
import frc.robot.subsystems.ShootingSubsystem.SpindexerSubsystem.IdxMode;
import frc.robot.subsystems.ShootingSubsystem.TurretSubsystem.TurretMode;
import java.util.function.Supplier;
import lib.ironpulse.io.MotorIO;
import lib.ironpulse.io.MotorInputsAutoLogged;
import lib.ironpulse.subsystem.position.PositionMotorSubsystem;
import lib.ironpulse.subsystem.velocity.VelocityMotorSubsystem;

public class ShootingSuperstructure {
    private final TurretSubsystem turret;
    private final PositionMotorSubsystem<MotorInputsAutoLogged, MotorIO, Angle> hood;
    private final VelocityMotorSubsystem<MotorInputsAutoLogged, MotorIO> shooter;
    private final SpindexerSubsystem idx;
    private final ShotCalculator shotCalculator;

    public ShootingSuperstructure(
            TurretSubsystem turret,
            PositionMotorSubsystem<MotorInputsAutoLogged, MotorIO, Angle> hood,
            VelocityMotorSubsystem<MotorInputsAutoLogged, MotorIO> shooter,
            SpindexerSubsystem idx,
            ShotCalculator shotCalculator) {
        this.turret = turret;
        this.hood = hood;
        this.shooter = shooter;
        this.idx = idx;
        this.shotCalculator = shotCalculator;
    }

    public void setDefaultCommand() {
        turret.setDefaultCommand(turret.runTurretTargetLoop());
        idx.setDefaultCommand();
        shooter.setDefaultCommand(
                shooter.runVelocity(
                        () -> RotationsPerSecond.of(ShooterParamsNT.idleVelRPS.getValue())));
    }

    public ShotFrame computeFrame(ShotCalculator.TargetMode targetMode, double shotDelaySec) {
        return shotCalculator.computeShotFrame(targetMode, shotDelaySec);
    }

    public Command runFrame(Supplier<ShotFrame> frame, TurretMode mode) {
        return Commands.parallel(
                turret.setTurretPoseWorld(() -> frame.get().turretAngleWorld(), mode)
                        .andThen(turret.runTurretTargetLoop()),
                hood.runPosition(() -> frame.get().hoodAngle()),
                shooter.runVelocity(() -> frame.get().shooterVelocity()));
    }

    public Command runFrame(Supplier<ShotFrame> frame, TurretMode mode, IdxMode idxMode) {
        return Commands.parallel(
                turret.setTurretPoseWorld(() -> frame.get().turretAngleWorld(), mode)
                        .andThen(turret.runTurretTargetLoop()),
                hood.runPosition(() -> frame.get().hoodAngle()),
                shooter.runVelocity(() -> frame.get().shooterVelocity()),
                idx.runMode(() -> readyToShoot() ? idxMode : IdxMode.OFF));
    }

    public boolean readyToShoot() {
        return turret.atGoal() && hood.positionAtGoal() && shooter.velocityAtGoal();
    }
}
