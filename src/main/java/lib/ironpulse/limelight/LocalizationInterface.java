package lib.ironpulse.limelight;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N4;

public interface LocalizationInterface {
    // any swerve that implements this function can be used in LimelightSubsystem.
    void addVisionMeasurement(
            Pose3d visionRobotPoseMeters,
            double timestampSeconds,
            Matrix<N4, N1> visionMeasurementStdDevs);
}
