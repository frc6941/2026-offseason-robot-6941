package frc.robot.subsystems.ShootingSubsystem;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Configs.ShotCalculatorParamsNT;
import frc.robot.subsystems.Configs.ShooterParamsNT;
import frc.robot.subsystems.ShootingSubsystem.SpindexerSubsystem.IdxMode;
import frc.robot.subsystems.ShootingSubsystem.TurretSubsystem.TurretMode;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;

import lib.ironpulse.io.MotorIO;
import lib.ironpulse.io.MotorInputsAutoLogged;
import lib.ironpulse.subsystem.position.PositionMotorSubsystem;
import lib.ironpulse.subsystem.velocity.VelocityMotorSubsystem;
import lombok.Getter;

public class ShootingSuperstructure {
    private final TurretSubsystem turret;
    private final PositionMotorSubsystem<MotorInputsAutoLogged, MotorIO, Angle> hood;
    private final VelocityMotorSubsystem<MotorInputsAutoLogged, MotorIO> shooter;
    private final SpindexerSubsystem idx;
    @Getter @AutoLogOutput(key= "ShootingSuperstructure/cmdFrame")
    private ShotFrame cmdFrame = new ShotFrame(
            Degrees.of(0.0), 
            Degrees.of(0.0), 
            MetersPerSecond.of(0.0));

    public ShootingSuperstructure(
            TurretSubsystem turret,
            PositionMotorSubsystem<MotorInputsAutoLogged, MotorIO, Angle> hood,
            VelocityMotorSubsystem<MotorInputsAutoLogged, MotorIO> shooter,
            SpindexerSubsystem idx
        ) {
        this.turret = turret;
        this.hood = hood;
        this.shooter = shooter;
        this.idx = idx;
    }

    public void setDefaultCommand() {
        turret.setDefaultCommand(turret.runTurretTargetLoop());
        idx.setDefaultCommand();
        shooter.setDefaultCommand(
                shooter.runVelocity(
                        () -> RotationsPerSecond.of(ShooterParamsNT.idleVelRPS.getValue())));
    }

    public Command runFrame(Supplier<ShotFrame> frame, TurretMode mode) {
        return Commands.parallel(
                Commands.runOnce(() -> cmdFrame = frame.get()),
                turret.setTurretPoseWorld(() -> frame.get().turretAngleWorld(), mode)
                        .andThen(turret.runTurretTargetLoop()),
                hood.runPosition(
                        () -> {
                            ShotFrame target = frame.get();
                            double modelDeg = target.hoodAngle().in(Degrees);
                            double bbaDeg =
                                    ShotCalculatorParamsNT.hoodB.getValue() * modelDeg
                                            + ShotCalculatorParamsNT.hoodC.getValue();
                            return Degrees.of(bbaDeg);
                        }),
                shooter.runVelocity(
                        () -> {
                            ShotFrame target = frame.get();
                            double muzzleSpeed = target.muzzleSpeed().in(MetersPerSecond);
                            double rpm =
                                    ShotCalculatorParamsNT.rpmA.getValue() * muzzleSpeed
                                            + ShotCalculatorParamsNT.rpmC.getValue();
                            return RotationsPerSecond.of(rpm / 60.0);
                        }));
    }

    public Command runFrame(Supplier<ShotFrame> frame, TurretMode mode, IdxMode idxMode) {
        return Commands.parallel(
                runFrame(frame, mode),
                idx.runMode(() -> readyToShoot() ? idxMode : IdxMode.OFF));
    }

    @AutoLogOutput(key= "ShootingSuperstructure/readyToShoot")
    public boolean readyToShoot() {
        return turret.atGoal() && hood.positionAtGoal() && shooter.velocityAtGoal();
    }

}
