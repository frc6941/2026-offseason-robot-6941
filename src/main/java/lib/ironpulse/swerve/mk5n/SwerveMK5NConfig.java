package lib.ironpulse.swerve.mk5n;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Frequency;
import lib.ironpulse.swerve.SwerveConfig;
import lombok.experimental.SuperBuilder;

/**
 * MK5n Swerve Module Configuration.
 * 
 * MK5n SPECIFICATIONS:
 * - Drive gear ratio: R2 = 6.03:1 (14/54 * 32/25 * 15/30)
 * - Steer gear ratio: 287:11 (≈26.09:1)
 * - Steer motor: Kraken X44
 * - Drive motor: Kraken X60
 * - Drive stator current limit: 80A (Kraken X60)
 * - Steer stator current limit: 40A (Kraken X44)
 */
@SuperBuilder
public class SwerveMK5NConfig extends SwerveConfig {
    public Current driveStatorCurrentLimit;
    public Current steerStatorCurrentLimit;
    public Frequency odometryFrequency;
    public String canivoreCanBusName;
    public int pigeonId;
}
