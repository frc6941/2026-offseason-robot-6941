package frc.robot.subsystems.ShootingSubsystem;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotStateRecorder;
import frc.robot.subsystems.ShootingSubsystem.TurretSubsystem.TurretMode;
import java.util.function.Supplier;
import lib.ironpulse.subsystem.position.PositionMotorSubsystem;
import lib.ironpulse.subsystem.velocity.VelocityMotorSubsystem;

public class ShootingSuperstructure {
    private final TurretSubsystem turret;
    private final PositionMotorSubsystem<?, ?, Angle> hood;
    private final VelocityMotorSubsystem<?, ?> shooter;
    private final VelocityMotorSubsystem<?, ?> indexer;
    private final ShootingParametersTable parametersTable;

    public ShootingSuperstructure(
            TurretSubsystem turret,
            PositionMotorSubsystem<?, ?, Angle> hood,
            VelocityMotorSubsystem<?, ?> shooter,
            VelocityMotorSubsystem<?, ?> indexer,
            ShootingParametersTable parametersTable) {
        this.turret = turret;
        this.hood = hood;
        this.shooter = shooter;
        this.indexer = indexer;
        this.parametersTable = parametersTable;
    }

    public ShotFrame computeFrame(Supplier<Pose3d> targetPoseWorld) {
        Pose3d turretPoseWorld = RobotStateRecorder.getPoseWorldTurretCurrent();
        Translation3d delta =
                targetPoseWorld.get().getTranslation().minus(turretPoseWorld.getTranslation());
        double distanceMeters = Math.hypot(delta.getX(), delta.getY());

        ShootingParameters params = parametersTable.getParameters(distanceMeters);
        Angle turretAngleWorld = Radians.of(Math.atan2(delta.getY(), delta.getX()));
        Angle hoodAngle = Degrees.of(params.getBackboardAngleDegree());
        AngularVelocity shooterVelocity =
                RotationsPerSecond.of(params.getVelocityRpm() / 60.0);

        return new ShotFrame(turretAngleWorld, hoodAngle, shooterVelocity);
    }

    public Command runFrame(Supplier<ShotFrame> frame, TurretMode mode) {
        return Commands.parallel(
                turret.setTurretPoseWorld(() -> frame.get().turretAngleWorld(), mode)
                        .andThen(turret.runTurretTargetLoop()),
                hood.runPosition(() -> frame.get().hoodAngle()),
                shooter.runVelocity(() -> frame.get().shooterVelocity()));
    }

    public Command runFrame(
            Supplier<ShotFrame> frame,
            Supplier<AngularVelocity> feedVelocity,
            TurretMode mode) {
        return Commands.parallel(
                turret.setTurretPoseWorld(() -> frame.get().turretAngleWorld(), mode)
                        .andThen(turret.runTurretTargetLoop()),
                hood.runPosition(() -> frame.get().hoodAngle()),
                shooter.runVelocity(() -> frame.get().shooterVelocity()),
                indexer.runVelocity(
                        () -> readyToShoot() ? feedVelocity.get() : RotationsPerSecond.of(0.0)));
    }

    public boolean readyToShoot() {
        return turret.atGoal() && hood.positionAtGoal() && shooter.velocityAtGoal();
    }
}
