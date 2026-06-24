package frc.robot.subsystems;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;

public record ShotFrame(Angle turretAngleWorld, Angle hoodAngle, LinearVelocity muzzleSpeed) {}
