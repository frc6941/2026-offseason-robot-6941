package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.RobotConstants;
import lib.ironpulse.swerve.SwerveConfig;
import lib.ironpulse.swerve.SwerveLimit;
import lib.ironpulse.swerve.SwerveModuleLimit;
import lib.ironpulse.swerve.mk5n.SwerveMK5NConfig;
import lib.ironpulse.swerve.sim.SwerveSimConfig;
import lib.ntext.NTParameter;

/** Constants specific to the swerve drivetrain configuration. */
public final class SwerveConstants {
    public static final String kSwerveTag = "Swerve";
    public static final String kSwerveModuleTag = "Swerve/SwerveModule";
    public static final double kSwerveHalfWidth = 0.6 / 2.0;

    public static SwerveModuleLimit kDefaultSwerveModuleLimit =
            SwerveModuleLimit.builder()
                    // MK5n L2 defaults (drive = 6.03, steer = 287/11 approx. equals to 26.09, wheel = 4.0in)
                    // This matters most when translating while rotating: if this is too optimistic,
                    // azimuth lag makes
                    // the net velocity vector feel "disoriented" during rotation even with
                    // open-loop drive.
                    // v (mps) = 6000rpm / 60 / 6.03 * pi * 4.0in
                    .maxDriveVelocity(MetersPerSecond.of(5.29329707470519))
                    .maxDriveAcceleration(MetersPerSecondPerSecond.of(17.0))
                    // omega (rps) = 6000rpm / 60 / (287/11) approx. equals to 3.8333 rps
                    .maxSteerAngularVelocity(RotationsPerSecond.of(6000.0 / 60.0 / (287.0 / 11.0)))
                    // accelerate in 0.1s
                    .maxSteerAngularAcceleration(
                            RotationsPerSecondPerSecond.of(6000.0 / 60.0 / (287.0 / 11.0) / 0.1))
                    .build();
    public static SwerveLimit kDefaultSwerveLimit =
            SwerveLimit.builder()
                    .maxLinearVelocity(MetersPerSecond.of(4.5))
                    // prevents skidding, see orbit archive ytb channel open class for theory
                    .maxSkidAcceleration(MetersPerSecondPerSecond.of(30))
                    // must be smaller than 4.5 / (distModuleToCenter * sqrt(2)) to be actually
                    // effective
                    .maxAngularVelocity(DegreesPerSecond.of(600.0))
                    // accelerate in 0.2s, also must be smaller than the defined module limit to be
                    // actually effective
                    .maxAngularAcceleration(DegreesPerSecondPerSecond.of(2000.0))
                    .build();

    public static SwerveConfig.SwerveModuleConfig kModuleCompFL =
            SwerveConfig.SwerveModuleConfig.builder()
                    .name("FL")
                    .location(new Translation2d(kSwerveHalfWidth, kSwerveHalfWidth))
                    .driveMotorId(4)
                    .steerMotorId(3)
                    .encoderId(10)
                    .driveMotorEncoderOffset(Degree.of(0))
                    .steerMotorEncoderOffset(Rotations.of(-0.360107421875))
                    .driveInverted(true)
                    .steerInverted(false)
                    .encoderInverted(false)
                    .build();
    public static SwerveConfig.SwerveModuleConfig kModuleCompFR =
            SwerveConfig.SwerveModuleConfig.builder()
                    .name("FR")
                    .location(new Translation2d(kSwerveHalfWidth, -kSwerveHalfWidth))
                    .driveMotorId(6)
                    .steerMotorId(5)
                    .encoderId(11)
                    .driveMotorEncoderOffset(Degree.of(0))
                    .steerMotorEncoderOffset(Rotations.of(-0.3486328125))
                    .driveInverted(false)
                    .steerInverted(false)
                    .encoderInverted(false)
                    .build();
    public static SwerveConfig.SwerveModuleConfig kModuleCompBL =
            SwerveConfig.SwerveModuleConfig.builder()
                    .name("BL")
                    .location(new Translation2d(-kSwerveHalfWidth, kSwerveHalfWidth))
                    .driveMotorId(2)
                    .steerMotorId(1)
                    .encoderId(0)
                    .driveMotorEncoderOffset(Degree.of(0))
                    .steerMotorEncoderOffset(Rotations.of(0.12158203125))
                    .driveInverted(true)
                    .steerInverted(false)
                    .encoderInverted(false)
                    .build();
    public static SwerveConfig.SwerveModuleConfig kModuleCompBR =
            SwerveConfig.SwerveModuleConfig.builder()
                    .name("BR")
                    .location(new Translation2d(-kSwerveHalfWidth, -kSwerveHalfWidth))
                    .driveMotorId(8)
                    .steerMotorId(7)
                    .encoderId(20)
                    .driveMotorEncoderOffset(Degree.of(0))
                    .steerMotorEncoderOffset(Rotations.of(0.2900390625))
                    .driveInverted(false)
                    .steerInverted(false)
                    .encoderInverted(false)
                    .build();
    public static SwerveSimConfig kSimConfig =
            SwerveSimConfig.builder()
                    .name("Swerve")
                    .dtS(RobotConstants.LOOPER_DT)
                    .wheelDiameter(Inch.of(4.1))
                    .driveGearRatio(7.0)
                    .steerGearRatio(20.0)
                    .driveMotor(DCMotor.getKrakenX60Foc(1))
                    .driveMomentOfInertia(KilogramSquareMeters.of(0.04))
                    .driveStdDevPos(0.0000001)
                    .driveStdDevVel(0.000001)
                    .steerMotor(DCMotor.getKrakenX60Foc(1))
                    .steerMomentOfInertia(KilogramSquareMeters.of(0.01))
                    .steerStdDevPos(0.0000001)
                    .steerStdDevVel(0.000001)
                    .defaultSwerveLimit(kDefaultSwerveLimit)
                    .defaultSwerveModuleLimit(kDefaultSwerveModuleLimit)
                    .moduleConfigs(
                            new SwerveConfig.SwerveModuleConfig[] {
                                kModuleCompFL, kModuleCompFR, kModuleCompBL, kModuleCompBR
                            })
                    .build();

    public static SwerveMK5NConfig kRealConfig =
            SwerveMK5NConfig.builder()
                    .name("Swerve")
                    .dtS(RobotConstants.LOOPER_DT)
                    .wheelDiameter(Inch.of(4.0))
                    .driveGearRatio(6.03)
                    .steerGearRatio(287.0 / 11.0)
                    .defaultSwerveLimit(kDefaultSwerveLimit)
                    .defaultSwerveModuleLimit(kDefaultSwerveModuleLimit)
                    .moduleConfigs(
                            new SwerveConfig.SwerveModuleConfig[] {
                                kModuleCompFL, kModuleCompFR, kModuleCompBL, kModuleCompBR
                            })
                    .odometryFrequency(Hertz.of(100))
                    .driveStatorCurrentLimit(Amps.of(80))
                    .steerStatorCurrentLimit(Amps.of(80))
                    .canivoreCanBusName(RobotConstants.CANIVORE_CAN_BUS_NAME)
                    .pigeonId(RobotConstants.PIGEON_ID)
                    .build();

    @NTParameter(tableName = "Params" + "/" + kSwerveModuleTag)
    @SuppressWarnings("unused")
    private static final class SwerveModuleParams {
        private static final class Drive {
            static final double kP = 4;
            static final double kI = 0.05;
            static final double kD = 0.1;
            static final double kS = 1.3;
            // CTRE Slot0 kV for VelocityTorqueCurrentFOC with motor velocity units (rotor rps):
            // kV ~= 12V / (6000rpm / 60) = 0.12
            static final double kV = 0.12;
            static final double kA = 0.19;
            static final boolean isBrake = true;
        }

        private static final class Steer {
            static final double kP = 30;
            static final double kI = 0;
            static final double kD = 0.1;
            static final double kS = 0;
            static final boolean isBrake = true;
        }
    }
}
