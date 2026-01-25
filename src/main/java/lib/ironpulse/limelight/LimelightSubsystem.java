package lib.ironpulse.limelight;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.HashMap;
import java.util.Map;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class LimelightSubsystem extends SubsystemBase {
    private final HashMap<LimelightIO, LimelightIOInputsAutoLogged> ios = new HashMap<>();
    private final LimelightSubsystemConfig config;
    private final LocalizationInterface localization;
    private final DoubleSupplier yawVelocitySupplier;

    public LimelightSubsystem(
            LimelightSubsystemConfig config,
            LocalizationInterface localization,
            DoubleSupplier yawVelocitySupplier,
            LimelightIO... ios) {
        super("Limelight");
        this.config = config;
        this.localization = localization;
        this.yawVelocitySupplier = yawVelocitySupplier;
        for (LimelightIO io : ios) {
            this.ios.put(io, new LimelightIOInputsAutoLogged());
        }
    }

    /**
     * Calculates the standard deviation matrix using weighted reliability scores. For reliability
     * scores, 1 is the most reliable and 0 the least reliable. The calculated standard deviations
     * would be, at most, twice the default standard deviation.
     *
     * <p>For example, if default standard deviation is 0.7: score=0, final=1.4; score=1, final=0.7.
     *
     * @param reliability The reliability score from limelight input.
     * @return The standard deviation for localization.
     */
    private Matrix<N4, N1> getVisionStdDev(double reliability) {
        return VecBuilder.fill(
                config.xStdDev * (2 - reliability),
                config.yStdDev * (2 - reliability),
                config.zStdDev * (2 - reliability),
                config.angleStdDev * (2 - reliability));
    }

    private void addVisionMeasurement() {
        if (yawVelocitySupplier.getAsDouble() > 360) {
            // angular velocity > 360 degrees per second, too high to have a reliable estimate
            return;
        }
        for (LimelightIOInputsAutoLogged input : ios.values()) {
            localization.addVisionMeasurement(
                    input.pose, input.timestampSeconds, getVisionStdDev(input.reliability));
        }
    }

    @Override
    public void periodic() {
        for (Map.Entry<LimelightIO, LimelightIOInputsAutoLogged> entry : ios.entrySet()) {
            LimelightIO io = entry.getKey();
            LimelightIOInputsAutoLogged inputs = entry.getValue();

            io.updateInputs(inputs);
            Logger.processInputs("Subsystem/Limelight/" + io.getName(), inputs);
        }

        addVisionMeasurement();
    }
}
