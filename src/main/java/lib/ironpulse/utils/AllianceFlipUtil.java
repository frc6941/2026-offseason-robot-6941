package lib.ironpulse.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class AllianceFlipUtil {
    private static final Rotation2d ROT180 = Rotation2d.fromDegrees(180.0);
    public static final double fieldLength = Units.inchesToMeters(690.876);
    public static final double fieldWidth = Units.inchesToMeters(317);

    public static double apply(double x) {
        if (shouldFlip()) {
            return fieldLength - x;
        }
        return x;
    }

    public static Translation2d apply(Translation2d t) {
        if (shouldFlip()) {
            double x = fieldLength - t.getX();
            double y = fieldWidth - t.getY();
            return new Translation2d(x, y);
        }
        return t;
    }

    public static Translation2d applySpeed(Translation2d v) {
        if (shouldFlip()) {
            return new Translation2d(-v.getX(), -v.getY());
        }
        return v;
    }

    public static Rotation2d apply(Rotation2d r) {
        if (shouldFlip()) {
            return r.plus(ROT180);
        }
        return r;
    }

    public static Pose2d apply(Pose2d p) {
        if (shouldFlip()) {
            Translation2d t = apply(p.getTranslation());
            Rotation2d r = apply(p.getRotation());
            return new Pose2d(t, r);
        }
        return p;
    }

    public static Translation3d apply(Translation3d t3) {
        if (shouldFlip()) {
            double x = fieldLength - t3.getX();
            double y = fieldWidth - t3.getY();
            return new Translation3d(x, y, t3.getZ());
        }
        return t3;
    }

    public static boolean shouldFlip() {
        return !DriverStation.getAlliance().isPresent()
                || DriverStation.getAlliance().get() == Alliance.Red;
    }
}
