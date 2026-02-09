package frc.robot.subsystems.ShootingSubsystem;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotStateRecorder;
import frc.robot.subsystems.Configs.ShooterParamsNT;
import frc.robot.subsystems.Configs.ShotCalculatorParamsNT;
import frc.robot.subsystems.Configs.SpindexerModeParamsNT;
import frc.robot.subsystems.ShootingSubsystem.TurretSubsystem.TurretMode;
import java.util.function.Supplier;
import lib.ironpulse.io.MotorIO;
import lib.ironpulse.io.MotorInputsAutoLogged;
import lib.ironpulse.subsystem.position.PositionMotorSubsystem;
import lib.ironpulse.subsystem.velocity.VelocityMotorSubsystem;
import lib.ntext.NTParameter;
import org.littletonrobotics.junction.AutoLogOutput;

public class ShootingSuperstructure {
  private final TurretSubsystem turret;
  private final PositionMotorSubsystem<MotorInputsAutoLogged, MotorIO, Angle> hood;
  private final VelocityMotorSubsystem<MotorInputsAutoLogged, MotorIO> shooter;
  private final VelocityMotorSubsystem<MotorInputsAutoLogged, MotorIO> idx;

  public ShootingSuperstructure(
      TurretSubsystem turret,
      PositionMotorSubsystem<MotorInputsAutoLogged, MotorIO, Angle> hood,
      VelocityMotorSubsystem<MotorInputsAutoLogged, MotorIO> shooter,
      VelocityMotorSubsystem<MotorInputsAutoLogged, MotorIO> idx) {
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
    if (hoodB == 0.0 || rpmA == 0.0) {
      throw new IllegalStateException("ShotCalculatorParamsNT.hoodB and rpmA must be non-zero");
    }
    Angle modelHood = bba.minus(Degrees.of(ShotCalculatorParamsNT.hoodC.getValue())).div(hoodB);
    AngularVelocity shooterVel = shooter.getVelocity();

    double rpm = shooterVel.in(RotationsPerSecond) * 60.0;
    double muzzleSpeedMps = (rpm - ShotCalculatorParamsNT.rpmC.getValue()) / rpmA;
    return new ShotFrame(turretWorldRotation, modelHood, MetersPerSecond.of(muzzleSpeedMps));
  }

  public void setDefaultCommand() {
    turret.setDefaultCommand(turret.runTurretTargetLoop());
    idx.setDefaultCommand(idx.runVelocity(() -> getIdxSpeed(IdxMode.OFF)));
    shooter.setDefaultCommand(
        shooter.runVelocity(() -> RotationsPerSecond.of(ShooterParamsNT.idleVelRPS.getValue())));
  }

  public Command runFrame(Supplier<ShotFrame> frame, TurretMode mode) {
    return Commands.parallel(
        Commands.run(() -> RobotStateRecorder.setCmdFrame(frame.get())),
        turret.setTurretPoseWorld(() -> frame.get().turretAngleWorld(), mode),
        hood.runPosition(
            () -> {
              Angle modelAngle = frame.get().hoodAngle();
              Angle bba =
                  modelAngle
                      .times(ShotCalculatorParamsNT.hoodB.getValue())
                      .plus(Degrees.of(ShotCalculatorParamsNT.hoodC.getValue()));
              return bba;
            }),
        shooter.runVelocity(
            () -> {
              double rpmA = ShotCalculatorParamsNT.rpmA.getValue();
              double rpm =
                  rpmA * frame.get().muzzleSpeed().in(MetersPerSecond)
                      + ShotCalculatorParamsNT.rpmC.getValue();
              return RotationsPerSecond.of(rpm / 60.0);
            }));
  }

  public Command runFrame(Supplier<ShotFrame> frame, TurretMode mode, IdxMode idxMode) {
    return Commands.parallel(
        runFrame(frame, mode),
        idx.runVelocity(() -> getIdxSpeed(readyToShoot() ? idxMode : IdxMode.OFF)));
  }


  @AutoLogOutput(key = "ShootingSuperstructure/readyToShoot")
  public boolean readyToShoot() {
    return turret.atGoal() && hood.positionAtGoal() && shooter.velocityAtGoal();
  }

  public AngularVelocity getIdxSpeed(IdxMode idxMode) {
    return switch (idxMode) {
      case OFF -> RotationsPerSecond.of(SpindexerModeParamsNT.idleRPS.getValue());
      case FEED -> RotationsPerSecond.of(SpindexerModeParamsNT.feedRPS.getValue());
      case REVERSE -> RotationsPerSecond.of(SpindexerModeParamsNT.revRPS.getValue());
    };
  }

  public enum IdxMode {
    OFF,
    FEED,
    REVERSE
  }

}
