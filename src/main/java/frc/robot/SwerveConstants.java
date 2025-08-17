package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.VoltageUnit;
import lib.ironpulse.swerve.SwerveConfig;
import lib.ironpulse.swerve.SwerveLimit;
import lib.ironpulse.swerve.SwerveModuleLimit;
import lib.ironpulse.swerve.sim.SwerveSimConfig;
import lib.ironpulse.swerve.sjtu6.SwerveSJTU6Config;
import lib.ntext.NTParameter;

/**
   * Constants specific to the swerve drivetrain configuration.
   */
  public final class SwerveConstants {
    public static final String kSwerveTag = "Swerve";
    public static final String kSwerveModuleTag = "Swerve/SwerveModule";
    public static final double kSwerveHalfWidth = 0.6 / 2.0;
    public static SwerveModuleLimit kDefaultSwerveModuleLimit = SwerveModuleLimit.builder()
        // v (mps) = 6000.0 (foc max omega in rpm) / 60.0 / reduction (drive gear ratio 7) * circumference
        .maxDriveVelocity(MetersPerSecond.of(4.559797259))
        .maxDriveAcceleration(MetersPerSecondPerSecond.of(20.0))
        // omega (rps) = 6000.0 (foc max omega in rpm) / 60.0 / reduction
        .maxSteerAngularVelocity(RotationsPerSecond.of(6000.0 / 60.0 / 22.0))
        // accelerate in 0.1s 
        .maxSteerAngularAcceleration(RotationsPerSecondPerSecond.of(6000.0 / 60.0 / 22.0 / 0.05))
        .build();
    public static SwerveLimit kDefaultSwerveLimit = SwerveLimit.builder()
        .maxLinearVelocity(MetersPerSecond.of(4.5))
        .maxSkidAcceleration(MetersPerSecondPerSecond.of(15.0))
        // must be smaller than 4.5 / (dist * sqrt(2)) to be actually effective
        .maxAngularVelocity(DegreesPerSecond.of(700.0))
        // accelerate in 0.2s, also must be smaller than the defined module limit to be actually effective
        .maxAngularAcceleration(DegreesPerSecondPerSecond.of(2000.0))
        .build();


    public static SwerveConfig.SwerveModuleConfig kModuleCompFL = SwerveConfig.SwerveModuleConfig.builder()
        .name("FL")
        .location(new Translation2d(kSwerveHalfWidth, kSwerveHalfWidth))
        .driveMotorId(4)
        .steerMotorId(3)
        .encoderId(10)
        .driveMotorEncoderOffset(Degree.of(0))
        .steerMotorEncoderOffset(Rotations.of(-0.247802))
        .driveInverted(false)
        .steerInverted(true)
        .encoderInverted(false)
        .build();
    public static SwerveConfig.SwerveModuleConfig kModuleCompFR = SwerveConfig.SwerveModuleConfig.builder()
        .name("FR")
        .location(new Translation2d(kSwerveHalfWidth, -kSwerveHalfWidth))
        .driveMotorId(6)
        .steerMotorId(5)
        .encoderId(11)
        .driveMotorEncoderOffset(Degree.of(0))
        .steerMotorEncoderOffset(Rotations.of(-0.295898))
        .driveInverted(true)
        .steerInverted(true)
        .encoderInverted(false)
        .build();
    public static SwerveConfig.SwerveModuleConfig kModuleCompBL = SwerveConfig.SwerveModuleConfig.builder()
        .name("BL")
        .location(new Translation2d(-kSwerveHalfWidth, kSwerveHalfWidth))
        .driveMotorId(2)
        .steerMotorId(1)
        .encoderId(0)
        .driveMotorEncoderOffset(Degree.of(0))
        .steerMotorEncoderOffset(Rotations.of(0.04003906))
        .driveInverted(false)
        .steerInverted(true)
        .encoderInverted(false)
        .build();
    public static SwerveConfig.SwerveModuleConfig kModuleCompBR = SwerveConfig.SwerveModuleConfig.builder()
        .name("BR")
        .location(new Translation2d(-kSwerveHalfWidth, -kSwerveHalfWidth))
        .driveMotorId(8)
        .steerMotorId(7)
        .encoderId(20)
        .driveMotorEncoderOffset(Degree.of(0))
        .steerMotorEncoderOffset(Rotations.of(-0.3833007))
        .driveInverted(true)
        .steerInverted(true)
        .encoderInverted(false)
        .build();
    public static SwerveSimConfig kSimConfig = SwerveSimConfig.builder()
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
        .moduleConfigs(new SwerveConfig.SwerveModuleConfig[]{
            kModuleCompFL, kModuleCompFR, kModuleCompBL, kModuleCompBR
        })
        .build();
    
    public static SwerveSJTU6Config kRealConfig = SwerveSJTU6Config.builder()
        .name("Swerve")
        .dtS(RobotConstants.LOOPER_DT)
        .wheelDiameter(Inch.of(4.1))
        .driveGearRatio(6.7460317460317460317460317460317)
        .steerGearRatio(21.428571428571428571428571428571)
        .defaultSwerveLimit(kDefaultSwerveLimit)
        .defaultSwerveModuleLimit(kDefaultSwerveModuleLimit)
        .moduleConfigs(new SwerveConfig.SwerveModuleConfig[]{
            kModuleCompFL, kModuleCompFR, kModuleCompBL, kModuleCompBR
        })
        .odometryFrequency(Hertz.of(100))
        .driveStatorCurrentLimit(Amps.of(100))
        .steerStatorCurrentLimit(Amps.of(40))
        .canivoreCanBusName(RobotConstants.CANIVORE_CAN_BUS_NAME)
        .pigeonId(RobotConstants.PIGEON_ID)
        .build();


    @NTParameter(tableName = "Params" + "/" + kSwerveModuleTag)
    private final static class SwerveModuleParams {
      private final static class Drive {
        static final double kP = 1;
        static final double kI = 0;
        static final double kD = 0;
        static final double kS = 0;
        static final double kV = 0;
        static final double kA = 0;
        static final boolean isBrake = true;
      }

      private final static class Steer {
        static final double kP = 1;
        static final double kI = 0;
        static final double kD = 0;
        static final double kS = 0;
        static final boolean isBrake = true;
      }
    }
  }
