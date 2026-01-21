package frc.robot.subsystems.ShootingSubsystem;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public record ShotFrame(
        Angle turretAngleWorld,
        Angle hoodAngle, 
        AngularVelocity shooterVelocity) {}
