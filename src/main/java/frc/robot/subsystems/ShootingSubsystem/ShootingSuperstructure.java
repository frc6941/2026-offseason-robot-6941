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
import frc.robot.RobotStateRecorder;

public class ShootingSuperstructure {
    private final TurretSubsystem turret;
    private final PositionMotorSubsystem<MotorInputsAutoLogged, MotorIO, Angle> hood;
    private final VelocityMotorSubsystem<MotorInputsAutoLogged, MotorIO> shooter;
    private final SpindexerSubsystem idx;
    public void updateCurrentFrame() {
        var turretWorldRotation =
                RobotStateRecorder.getPoseWorldShotCurrent().toPose2d().getRotation();
        double bbaDeg = hood.getCurrPos().in(Degrees);
        double hoodB = ShotCalculatorParamsNT.hoodB.getValue();
        double modelHoodDeg =
                Math.abs(hoodB) < 1e-9
                        ? bbaDeg
                        : (bbaDeg - ShotCalculatorParamsNT.hoodC.getValue()) / hoodB;
        double rps = shooter.getVelocity().in(RotationsPerSecond);
        double rpmA = ShotCalculatorParamsNT.rpmA.getValue();
        double rpm = rps * 60.0;
        double muzzleSpeedMps =
                Math.abs(rpmA) < 1e-9
                        ? 0.0
                        : (rpm - ShotCalculatorParamsNT.rpmC.getValue()) / rpmA;
        RobotStateRecorder.setCurrentFrame(
                new ShotFrame(
                        Degrees.of(turretWorldRotation.getDegrees()),
                        Degrees.of(modelHoodDeg),
                        MetersPerSecond.of(muzzleSpeedMps)));
    }

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
                Commands.run(() -> RobotStateRecorder.setCmdFrame(frame.get())),
                turret.setTurretPoseWorld(() -> frame.get().turretAngleWorld(), mode),
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
