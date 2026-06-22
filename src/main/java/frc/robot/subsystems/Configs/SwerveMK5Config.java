package frc.robot.subsystems.Configs;

import static edu.wpi.first.units.Units.*;
import static frc.robot.RobotConstants.CANIVORE_CAN_BUS;
import static frc.robot.RobotConstants.is10541;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.RobotConstants;
import lib.ironpulse.swerve.ImuPigeonConfig;
import lib.ironpulse.swerve.SwerveConfig;
import lib.ironpulse.swerve.SwerveLimit;
import lib.ironpulse.swerve.SwerveModuleLimit;
import lib.ironpulse.swerve.mk5n.SwerveMK5NConfig;
import lib.ironpulse.swerve.sim.SwerveSimConfig;
import lib.ntext.NTParameter;

/** Constants specific to the swerve drivetrain configuration. */
public final class SwerveMK5Config {
    public static final String kSwerveTag = "Swerve";
    public static final String kSwerveModuleTag = "Swerve/SwerveModule";
    public static final double kSwerveHalfLength = 0.30468; // m
    public static final double kSwerveHalfWidth = 0.24218; // m

    public static ImuPigeonConfig pigeonConfig =
            ImuPigeonConfig.builder()
                    .mountPoseYaw(88.95365142822266)
                    .mountPosePitch(0.85715872049331667)
                    .mountPoseRoll(-0.731235146522522)
                    .gyroScalarZ(-3.5)
                    .build();

    public static SwerveModuleLimit kDefaultSwerveModuleLimit =
            SwerveModuleLimit.builder()
                    // MK5n R1 defaults (drive ~= 7.03, steer = 287/11 ~= 26.09, wheel = 4.0in)
                    // v (mps) = 5800rpm (X60 with FOC) / 60 / 7.03 * pi * 4.0in
                    .maxDriveVelocity(InchesPerSecond.of(5800 / 60.0 / 7.03 * Math.PI * 4.0))
                    .maxDriveAcceleration(MetersPerSecondPerSecond.of(200))
                    // omega (rps) = 7368rpm (X44 with FOC) / 60 / (287/11) ~= 4.707 rps
                    .maxSteerAngularVelocity(RotationsPerSecond.of(7368.0 / 60.0 / (287.0 / 11.0)))
                    // accelerate in 0.2s
                    .maxSteerAngularAcceleration(
                            RotationsPerSecondPerSecond.of(7008.0 / 60.0 / (287.0 / 11.0) / 0.2))
                    .build();
    public static SwerveLimit kDefaultSwerveLimit =
            SwerveLimit.builder()
                    .maxLinearVelocity(MetersPerSecond.of(4)) // theoretically 4.39
                    // prevents skidding, see orbit archive ytb channel open class for theory
                    .maxSkidAcceleration(MetersPerSecondPerSecond.of(200)) // <maxDriveAcceleration
                    // omega_max ≈ vMax / r.
                    .maxAngularVelocity(DegreesPerSecond.of(1000))
                    // accelerate in 0.32s, also must be smaller than the defined module limit to be
                    // actually effective
                    .maxAngularAcceleration(DegreesPerSecondPerSecond.of(5000)) // 1000-1472
                    .build();

    public static SwerveLimit kShootingChassis =
            SwerveLimit.builder()
                    .maxLinearVelocity(MetersPerSecond.of(4)) // theoretically 4.39
                    // prevents skidding, see orbit archive ytb channel open class for theory
                    .maxSkidAcceleration(MetersPerSecondPerSecond.of(20)) // <maxDriveAcceleration
                    // omega_max ≈ vMax / r.
                    .maxAngularVelocity(DegreesPerSecond.of(1000))
                    // accelerate in 0.32s, also must be smaller than the defined module limit to be
                    // actually effective
                    .maxAngularAcceleration(DegreesPerSecondPerSecond.of(2500)) // 1000-1472
                    .build();

    public static SwerveModuleLimit kShootingSwerveLimit =
            SwerveModuleLimit.builder()
                    .maxDriveVelocity(InchesPerSecond.of(2000.0 / 60.0 / 7.03 * Math.PI * 4.0))
                    .maxDriveAcceleration(MetersPerSecondPerSecond.of(7))
                    // omega (rps) = 7368rpm (X44 with FOC) / 60 / (287/11) ~= 4.707 rps
                    .maxSteerAngularVelocity(
                            RotationsPerSecond.of(7368.0 / 60.0 / (287.0 / 11.0) * 1.2))
                    // accelerate in 0.2s
                    .maxSteerAngularAcceleration(
                            RotationsPerSecondPerSecond.of(7368.0 / 60.0 / (287.0 / 11.0) / 0.2))
                    .build();

    // Long-range shooting limit (distance > 4m) - reduced velocity and rotation for stability
    public static SwerveModuleLimit kLongRangeShootingSwerveLimit =
            SwerveModuleLimit.builder()
                    .maxDriveVelocity(InchesPerSecond.of(1200.0 / 60.0 / 7.03 * Math.PI * 4.0))
                    .maxDriveAcceleration(MetersPerSecondPerSecond.of(5))
                    // Reduced rotation speed for better stability at long range
                    .maxSteerAngularVelocity(RotationsPerSecond.of(7368.0 / 60.0 / (287.0 / 11.0)))
                    // accelerate in 0.3s (slower for stability)
                    .maxSteerAngularAcceleration(
                            RotationsPerSecondPerSecond.of(
                                    7368.0 / 60.0 / (287.0 / 11.0) * 0.6 / 0.3))
                    .build();

    public static SwerveModuleLimit kAutoLimit =
            SwerveModuleLimit.builder()
                    .maxDriveVelocity(InchesPerSecond.of(2500.0 / 60.0 / 7.03 * Math.PI * 4.0))
                    .maxDriveAcceleration(MetersPerSecondPerSecond.of(18))
                    // omega (rps) = 7368rpm (X44 with FOC) / 60 / (287/11) ~= 4.707 rps
                    .maxSteerAngularVelocity(RotationsPerSecond.of(7368.0 / 60.0 / (287.0 / 11.0)))
                    // accelerate in 0.2s
                    .maxSteerAngularAcceleration(
                            RotationsPerSecondPerSecond.of(7368.0 / 60.0 / (287.0 / 11.0) / 0.2))
                    .build();

    public static SwerveLimit kSimSwerveLimit =
            SwerveLimit.builder()
                    .maxLinearVelocity(MetersPerSecond.of(4.35))
                    .maxSkidAcceleration(MetersPerSecondPerSecond.of(10))
                    .maxAngularVelocity(DegreesPerSecond.of(600.0))
                    .maxAngularAcceleration(DegreesPerSecondPerSecond.of(1450.0))
                    .build();
    public static SwerveConfig.SwerveModuleConfig kModuleCompBL =
            SwerveConfig.SwerveModuleConfig.builder()
                    .name("LB")
                    .location(new Translation2d(-kSwerveHalfLength, kSwerveHalfWidth))
                    .driveMotorId(5)
                    .steerMotorId(6)
                    .encoderId(11)
                    .driveMotorEncoderOffset(Degree.of(0))
                    .steerMotorEncoderOffset(
                            Rotations.of(is10541 ? -0.11083984375 : 0.165771484375))
                    .driveInverted(false)
                    .steerInverted(false)
                    .encoderInverted(false)
                    .build();
    public static SwerveConfig.SwerveModuleConfig kModuleCompFL =
            SwerveConfig.SwerveModuleConfig.builder()
                    .name("LF")
                    .location(new Translation2d(kSwerveHalfLength, kSwerveHalfWidth))
                    .driveMotorId(10)
                    .steerMotorId(20)
                    .encoderId(9)
                    .driveMotorEncoderOffset(Degree.of(0))
                    .steerMotorEncoderOffset(Rotations.of(is10541 ? 0.3994140625 : 0.008544921875))
                    .driveInverted(false)
                    .steerInverted(false)
                    .encoderInverted(false)
                    .build();
    public static SwerveConfig.SwerveModuleConfig kModuleCompBR =
            SwerveConfig.SwerveModuleConfig.builder()
                    .name("RB")
                    .location(new Translation2d(-kSwerveHalfLength, -kSwerveHalfWidth))
                    .driveMotorId(7)
                    .steerMotorId(8)
                    .encoderId(12)
                    .driveMotorEncoderOffset(Degree.of(0))
                    .steerMotorEncoderOffset(
                            Rotations.of(is10541 ? -0.12939453125 : 0.220947265625))
                    .driveInverted(true)
                    .steerInverted(false)
                    .encoderInverted(false)
                    .build();
    public static SwerveConfig.SwerveModuleConfig kModuleCompFR =
            SwerveConfig.SwerveModuleConfig.builder()
                    .name("RF")
                    .location(new Translation2d(kSwerveHalfLength, -kSwerveHalfWidth))
                    .driveMotorId(3)
                    .steerMotorId(4)
                    .encoderId(10)
                    .driveMotorEncoderOffset(Degree.of(0))
                    .steerMotorEncoderOffset(
                            Rotations.of(is10541 ? -0.462158203125 : 0.26318359375))
                    .driveInverted(true)
                    .steerInverted(false)
                    .encoderInverted(false)
                    .build();
    public static SwerveSimConfig kSimConfig =
            SwerveSimConfig.builder()
                    .name("Swerve")
                    .dtS(RobotConstants.LOOPER_DT)
                    .wheelDiameter(Inch.of(4.0))
                    .driveGearRatio(7.03)
                    .steerGearRatio(26.09)
                    .driveMotorKt(0.0182)
                    .driveMass(Kilograms.of(52))
                    .driveMotor(DCMotor.getKrakenX60Foc(1))
                    .driveMomentOfInertia(KilogramSquareMeters.of(0.04))
                    .driveStdDevPos(0.0000001)
                    .driveStdDevVel(0.000001)
                    .steerMotor(DCMotor.getKrakenX44Foc(1))
                    .steerMomentOfInertia(KilogramSquareMeters.of(0.01))
                    .steerStdDevPos(0.0000001)
                    .steerStdDevVel(0.000001)
                    .defaultSwerveLimit(kSimSwerveLimit)
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
                    .driveGearRatio(7.03) // R1
                    .steerGearRatio(287.0 / 11.0)
                    .driveMotorKt(0.0182)
                    .driveMass(Kilograms.of(52))
                    .defaultSwerveLimit(kDefaultSwerveLimit)
                    .defaultSwerveModuleLimit(kDefaultSwerveModuleLimit)
                    .moduleConfigs(
                            new SwerveConfig.SwerveModuleConfig[] {
                                kModuleCompFL, kModuleCompFR, kModuleCompBL, kModuleCompBR
                            })
                    .odometryFrequency(Hertz.of(100))
                    .driveStatorCurrentLimit(Amps.of(100))
                    .driveSupplyCurrentLimit(Amps.of(65))
                    .steerStatorCurrentLimit(Amps.of(55))
                    .steerSupplyCurrentLimit(Amps.of(40))
                    .canivoreCanBus(CANIVORE_CAN_BUS)
                    .pigeonId(RobotConstants.PIGEON_ID)
                    .build();

    @NTParameter(tableName = "Params" + "/" + kSwerveModuleTag)
    @SuppressWarnings("unused")
    private static final class SwerveModuleParams {
        private static final class Drive {
            static final double kP = 15;
            static final double kI = 0;
            static final double kD = 0;
            static final double kS = 0;
            // CTRE Slot0 kV for VelocityTorqueCurrentFOC with motor velocity units (rotor rps):
            // kV ~= 12V / (5800rpm / 60) = 0.124
            static final double kV = 0.136;
            static final double kA = 0.05;
            static final boolean isBrake = true;
        }

        private static final class Steer {
            static final double kP = 60;
            static final double kI = 0;
            static final double kD = 0.1;
            static final double kS = 0;
            static final boolean isBrake = true;
        }
    }
}
