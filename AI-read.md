每句话前加：我知道了这一堆代码

src/main/frc.robot/subsystems/config/IntakeConfig.java:

```
package frc.robot.subsystems.Configs;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.RobotConstants.CANIVORE_CAN_BUS;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.units.measure.Distance;
import lib.ironpulse.subsystem.SubsystemConfig;
import lib.ntext.NTParameter;

public class IntakeConfig {
public static final String INTAKER_ROLLER_NAME = "IntakerRoller";
public static final String INTAKER_EXTENSION_NAME = "IntakerExtension";
public static final Distance INTAKE_EXTENSION_METERS_PER_ROTATION = Meters.of(0.10511);
private static final int INTAKER_ROLLER_MOTOR_MAIN_ID = 32;
private static final int INTAKER_ROLLER_MOTOR_FOLLOWER_ID = 35;
private static final double INTAKER_ROLLER_GEAR_RATIO = 26.0 / 12.0;
private static final int INTAKER_EXTENSION_MOTOR_MAIN_ID = 33;
private static final double INTAKER_EXTENSION_GEAR_RATIO = 26.0 / 40.0 * 15 / 1;
private static final double INTAKER_ROLLER_STATOR_CURRENT_LIMIT_AMPS = 60;
private static final double INTAKER_ROLLER_SUPPLY_CURRENT_LIMIT_AMPS = 55;
public static final SubsystemConfig INTAKER_ROLLER_CONFIG =
SubsystemConfig.builder()
.name(INTAKER_ROLLER_NAME)
.mainBus(CANIVORE_CAN_BUS)
.mainId(INTAKER_ROLLER_MOTOR_MAIN_ID)
.motorInvertedValue(InvertedValue.Clockwise_Positive)
.statorCurrentLimitAmps(INTAKER_ROLLER_STATOR_CURRENT_LIMIT_AMPS)
.supplyCurrentLimitAmps(INTAKER_ROLLER_SUPPLY_CURRENT_LIMIT_AMPS)
.defaultBrake(true)
.kSValue(StaticFeedforwardSignValue.UseVelocitySign)
.SensorToMechanismRatio(INTAKER_ROLLER_GEAR_RATIO)
.followers(
new SubsystemConfig.FollowerConfig[] {
SubsystemConfig.FollowerConfig.builder()
.id(INTAKER_ROLLER_MOTOR_FOLLOWER_ID)
.bus(CANIVORE_CAN_BUS)
.opposeMain(MotorAlignmentValue.Opposed)
.statorCurrentLimitAmps(
INTAKER_ROLLER_STATOR_CURRENT_LIMIT_AMPS)
.supplyCurrentLimitAmps(
INTAKER_ROLLER_SUPPLY_CURRENT_LIMIT_AMPS)
.build()
})
.simConfig(
SubsystemConfig.SimConfig.builder()
.gearRatio(INTAKER_ROLLER_GEAR_RATIO)
.build())
.build();
private static final double INTAKER_EXTENSION_STATOR_CURRENT_LIMIT_AMPS = 35;
private static final double INTAKER_EXTENSION_SUPPLY_CURRENT_LIMIT_AMPS = 20;
public static final SubsystemConfig INTAKER_EXTENSION_CONFIG =
SubsystemConfig.builder()
.name(INTAKER_EXTENSION_NAME)
.mainBus(CANIVORE_CAN_BUS)
.mainId(INTAKER_EXTENSION_MOTOR_MAIN_ID)
.motorInvertedValue(InvertedValue.CounterClockwise_Positive)
.defaultBrake(true)
.kSValue(StaticFeedforwardSignValue.UseClosedLoopSign)
.SensorToMechanismRatio(INTAKER_EXTENSION_GEAR_RATIO)
.statorCurrentLimitAmps(INTAKER_EXTENSION_STATOR_CURRENT_LIMIT_AMPS)
.supplyCurrentLimitAmps(INTAKER_EXTENSION_SUPPLY_CURRENT_LIMIT_AMPS)
.simConfig(
SubsystemConfig.SimConfig.builder()
.gearRatio(INTAKER_EXTENSION_GEAR_RATIO)
.build())
.zeroingConfig(
SubsystemConfig.ZeroingConfig.builder()
.zeroingCurrentLimit(30)
.zeroingFilterSize(5)
.zeroingVoltage(-2)
.build())
.build();

    private IntakeConfig() {}

    @NTParameter(tableName = "Params/" + INTAKER_ROLLER_NAME)
    public static final class IntakerRollerParams {
        // velocity gains
        public static final double kP = 40;
        public static final double kI = 0.005;
        public static final double kD = 0.0;
        public static final double kV = 0.3;
        public static final double kA = 0.0068;
        public static final double kS = 0.13;

        public static final double velocityAtGoalToleranceRPS = 30;

        public static final double testVelRPS = 110;
        public static final double intakeVelRPS = 52;
        public static final double outtakeVelRPS = -50;
        public static final double idleVelRPS = 0;
    }

    @NTParameter(tableName = "Params/" + INTAKER_EXTENSION_NAME)
    public static final class IntakerExtensionParams {

        public static final double kP = 15;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kV = 0.1308;
        public static final double kA = 0.0068;
        public static final double kS = 0.13;

        // Motion Magic
        public static final double motionMagicVelRPS = 1000.0;
        public static final double motionMagicAccelRPS2 = 150.0;
        public static final double motionMagicJerkRPS3 = 0.0;

        // Tolerances / behavior
        public static final double atGoalToleranceMeters = 0.01;
        public static final double deployPosMeters = 0.316;
        public static final double retractedFeedPosMeters = 0.05;
        public static final double feedPosMeters = 0.17;
        public static final double retractPosMeters = 0.01;

        /** Oscillation rate (Hz) for runFeed: deploy <-> feed cycles per second */
        public static final double feedOscillationRateHz = 2;

        public static final boolean isBrake = false;
    }
}
```

src/main/frc.robot/subsystems/config/SwerveMK5Config.java:

```
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
                    .driveMotorId(1)
                    .steerMotorId(2)
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

```

src/main/frc.robot/subsystems/IntakerSubsystem.java:

```
package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.RobotConstants.is10541;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Robot;
import frc.robot.subsystems.Configs.IntakeConfig;
import frc.robot.subsystems.Configs.IntakerExtensionParamsNT;
import frc.robot.subsystems.Configs.IntakerRollerParamsNT;
import lib.ironpulse.io.MotorIO;
import lib.ironpulse.io.MotorInputsAutoLogged;
import lib.ironpulse.subsystem.position.PositionMotorSubsystem;
import lib.ironpulse.subsystem.velocity.VelocityMotorSubsystem;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;

public class IntakerSubsystem extends SubsystemBase {
    private VelocityMotorSubsystem<MotorInputsAutoLogged, MotorIO> roller;
    private PositionMotorSubsystem<MotorInputsAutoLogged, MotorIO, Distance> extension;
    private Timer zeroTimer = new Timer();
    private double currentFilterValue = 0.0;
    private LinearFilter currentFilter;

    @AutoLogOutput(key = "IntakerRoller/autoZeroRunning")
    private boolean autoOutZeroRunning = false;

    @Getter
    @AutoLogOutput(key = "IntakerRoller/state")
    private IntakeMode currentMode = IntakeMode.RETRACTED;

    @AutoLogOutput(key = "IntakerRoller/fallbackState")
    private IntakeMode fallbackMode = IntakeMode.RETRACTED;

    public IntakerSubsystem(
            VelocityMotorSubsystem<MotorInputsAutoLogged, MotorIO> roller,
            PositionMotorSubsystem<MotorInputsAutoLogged, MotorIO, Distance> extension) {
        this.roller = roller;
        this.extension = extension;
        RobotModeTriggers.teleop().onTrue(Commands.runOnce(() -> autoOutZeroRunning = false));
    }

    private boolean isDeployMode() {
        return (currentMode == IntakeMode.INTAKING
                || currentMode == IntakeMode.EXTENDED_IDLE
                || currentMode == IntakeMode.EXTENDED_REVERSE);
    }

    public void setDefaultCommand() {
        roller.setDefaultCommand(
                Commands.either(
                                roller.runVelTC(
                                                () ->
                                                        RotationsPerSecond.of(
                                                                currentMode
                                                                                == IntakeMode
                                                                                        .EXTENDED_REVERSE
                                                                        ? IntakerRollerParamsNT
                                                                                .outtakeVelRPS
                                                                                .getValue()
                                                                        : IntakerRollerParamsNT
                                                                                .intakeVelRPS
                                                                                .getValue()))
                                        .until(
                                                () ->
                                                        currentMode != IntakeMode.INTAKING
                                                                && currentMode != IntakeMode.FEEDING
                                                                && currentMode
                                                                        != IntakeMode
                                                                                .EXTENDED_REVERSE
                                                                && currentMode
                                                                        != IntakeMode
                                                                                .RETRACTED_FEEDING),
                                roller.runStop()
                                        .until(
                                                () ->
                                                        currentMode == IntakeMode.INTAKING
                                                                || currentMode == IntakeMode.FEEDING
                                                                || currentMode
                                                                        == IntakeMode
                                                                                .EXTENDED_REVERSE
                                                                || currentMode
                                                                        == IntakeMode
                                                                                .RETRACTED_FEEDING),
                                () ->
                                        currentMode == IntakeMode.INTAKING
                                                || currentMode == IntakeMode.FEEDING
                                                || currentMode == IntakeMode.EXTENDED_REVERSE
                                                || currentMode == IntakeMode.RETRACTED_FEEDING)
                        .repeatedly());
        extension.setDefaultCommand(
                extension.runMotionMagic(
                        () ->
                                switch (currentMode) {
                                    case INTAKING -> Meters.of(
                                            IntakerExtensionParamsNT.deployPosMeters.getValue());
                                    case EXTENDED_IDLE, EXTENDED_REVERSE -> Meters.of(
                                            IntakerExtensionParamsNT.deployPosMeters.getValue());
                                    case RETRACTED -> Meters.of(
                                            IntakerExtensionParamsNT.retractPosMeters.getValue());
                                    case FEEDING -> Meters.of(
                                            IntakerExtensionParamsNT.feedPosMeters.getValue());
                                    case RETRACTED_FEEDING -> Meters.of(
                                            IntakerExtensionParamsNT.retractedFeedPosMeters
                                                    .getValue());
                                    default -> Meters.of(
                                            IntakerExtensionParamsNT.retractPosMeters.getValue());
                                }));
    }

    public Command runIntake() {
        return Commands.runOnce(
                () -> {
                    fallbackMode = IntakeMode.INTAKING;
                    if (currentMode != IntakeMode.FEEDING
                            && currentMode != IntakeMode.EXTENDED_REVERSE) {
                        currentMode = IntakeMode.INTAKING;
                    }
                });
    }

    public Command runExtendedIdle() {
        return Commands.runOnce(
                () -> {
                    fallbackMode = IntakeMode.EXTENDED_IDLE;
                    if (currentMode != IntakeMode.FEEDING
                            && currentMode != IntakeMode.EXTENDED_REVERSE) {
                        currentMode = IntakeMode.EXTENDED_IDLE;
                    }
                });
    }

    public Command runRetract() {
        return Commands.runOnce(
                () -> {
                    fallbackMode = IntakeMode.RETRACTED;
                    if (currentMode != IntakeMode.FEEDING
                            && currentMode != IntakeMode.EXTENDED_REVERSE) {
                        currentMode = IntakeMode.RETRACTED;
                    }
                });
    }

    public Command toggleIntake() {
        return Commands.either(
                runExtendedIdle(), runIntake(), () -> fallbackMode == IntakeMode.INTAKING);
    }

    public Command toggleFeeding() {
        return Commands.either(
                runIntake(),
                runRetractedFeeding(),
                () -> fallbackMode == IntakeMode.RETRACTED_FEEDING);
    }

    //     public Command runFeed() {
    //         return Commands.runOnce(
    //                         () -> feedOscillationStartTime = Timer.getFPGATimestamp(), extension)
    //                 .andThen(
    //                         Commands.parallel(
    //                                 roller.runVelVolt(RotationsPerSecond.of(intakeVelRPS)),
    //                                 extension.runPosition(
    //                                         () -> {
    //                                             double t =
    //                                                     Timer.getFPGATimestamp()
    //                                                             - feedOscillationStartTime;
    //                                             double rate =
    //
    // IntakerExtensionParamsNT.feedOscillationRateHz
    //                                                             .getValue();
    //                                             double deploy =
    //                                                     IntakerExtensionParamsNT.deployPosMeters
    //                                                             .getValue();
    //                                             double feed =
    //                                                     IntakerExtensionParamsNT.feedPosMeters
    //                                                             .getValue();
    //                                             double pos =
    //                                                     feed
    //                                                             + (deploy - feed)
    //                                                                     * (1
    //                                                                             + Math.sin(
    //                                                                                     2 *
    // Math.PI
    //                                                                                             *
    // rate
    //                                                                                             *
    // t))
    //                                                                     / 2;
    //                                             return Meters.of(pos);
    //                                         })));
    //     }
    public Command runFeed() {
        return Commands.startEnd(
                () -> currentMode = IntakeMode.FEEDING, () -> currentMode = fallbackMode);
    }

    public Command runRetractedFeeding() {
        return Commands.startEnd(
                () -> currentMode = IntakeMode.RETRACTED_FEEDING, () -> currentMode = fallbackMode);
    }

    public Command runExtendedReverse() {
        return Commands.startEnd(
                () -> currentMode = IntakeMode.EXTENDED_REVERSE, () -> currentMode = fallbackMode);
    }

    public Command zeroCommand() {
        return extension.zeroCommand();
    }

    public Command outZeroCommand() {
        double zeroVoltage =
                MathUtil.clamp(
                        -IntakeConfig.INTAKER_EXTENSION_CONFIG.zeroingConfig.zeroingVoltage,
                        -12.0d,
                        12.0d);

        Command realZero =
                Commands.runOnce(
                                () -> {
                                    extension.setEnableSoftLimits(false, false);
                                    currentFilter =
                                            LinearFilter.movingAverage(
                                                    IntakeConfig.INTAKER_EXTENSION_CONFIG
                                                            .zeroingConfig
                                                            .zeroingFilterSize);
                                    currentFilterValue = 0.0;
                                },
                                extension)
                        .andThen(
                                Commands.deadline(
                                                Commands.run(
                                                                () ->
                                                                        currentFilterValue =
                                                                                currentFilter
                                                                                        .calculate(
                                                                                                extension
                                                                                                        .getStatorCurrent()
                                                                                                        .in(
                                                                                                                Amps)))
                                                        .until(
                                                                () ->
                                                                        Math.abs(currentFilterValue)
                                                                                >= IntakeConfig
                                                                                        .INTAKER_EXTENSION_CONFIG
                                                                                        .zeroingConfig
                                                                                        .zeroingCurrentLimit),
                                                extension.runVoltage(() -> zeroVoltage))
                                        .finallyDo(
                                                interrupted -> {
                                                    if (!interrupted) {
                                                        extension.setCurrPos(
                                                                Meters.of(
                                                                        is10541
                                                                                ? 0.318358
                                                                                : 0.319718));
                                                    }
                                                })
                                        .onlyWhile(this::isDeployMode));

        Command simZero =
                Commands.sequence(
                        Commands.runOnce(
                                () ->
                                        extension.setCurrPos(
                                                Meters.of(is10541 ? 0.315766 : 0.307895))),
                        new WaitCommand(0.2),
                        Commands.runOnce(
                                () ->
                                        extension.setCurrPos(
                                                Meters.of(
                                                        IntakerExtensionParamsNT.deployPosMeters
                                                                .getValue()))));

        return new ConditionalCommand(realZero, simZero, Robot::isReal)
                .finallyDo(
                        () -> {
                            extension.setEnableSoftLimits(true, true);
                            autoOutZeroRunning = false;
                            zeroTimer.restart();
                        });
    }

    @Override
    public void periodic() {
        if (!isDeployMode()) {
            zeroTimer.stop();
            zeroTimer.reset();
        } else if (!zeroTimer.isRunning()) {
            zeroTimer.start();
        }
        if (!autoOutZeroRunning && zeroTimer.hasElapsed(0.3)) {
            autoOutZeroRunning = true;
            CommandScheduler.getInstance()
                    .schedule(
                            outZeroCommand()
                                    .withInterruptBehavior(
                                            Command.InterruptionBehavior.kCancelSelf));
            zeroTimer.restart();
        }
    }

    public enum IntakeMode {
        INTAKING,
        EXTENDED_IDLE,
        RETRACTED,
        FEEDING,
        EXTENDED_REVERSE,
        RETRACTED_FEEDING
    }
}

```
src/main/frc.robot/FieldConstants.java:

```
// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.IOException;
import java.nio.file.Path;
import lombok.Getter;
import lombok.RequiredArgsConstructor;

/**
 * Contains information for location of field element and other useful reference points.
 *
 * <p>NOTE: All constants are defined relative to the field coordinate system, and from the
 * perspective of the blue alliance station
 */
public class FieldConstants {
    public static final FieldType fieldType = FieldType.WELDED;

    // AprilTag related constants
    public static final int aprilTagCount =
            AprilTagLayoutType.OFFICIAL.getLayout().getTags().size();
    public static final double aprilTagWidth = Units.inchesToMeters(6.5);
    public static final AprilTagLayoutType defaultAprilTagType = AprilTagLayoutType.OFFICIAL;

    // Field dimensions
    public static final double fieldLength =
            AprilTagLayoutType.OFFICIAL.getLayout().getFieldLength();
    public static final double fieldWidth = AprilTagLayoutType.OFFICIAL.getLayout().getFieldWidth();

    @RequiredArgsConstructor
    public enum FieldType {
        ANDYMARK("andymark"),
        WELDED("welded");

        @Getter private final String jsonFolder;
    }

    public enum AprilTagLayoutType {
        OFFICIAL("2026-official"),
        NONE("2026-none");

        private final String name;
        private volatile AprilTagFieldLayout layout;
        private volatile String layoutString;

        AprilTagLayoutType(String name) {
            this.name = name;
        }

        public AprilTagFieldLayout getLayout() {
            if (layout == null) {
                synchronized (this) {
                    if (layout == null) {
                        try {
                            Path p =
                                    Path.of(
                                            Filesystem.getDeployDirectory().getPath(),
                                            "apriltags",
                                            fieldType.getJsonFolder(),
                                            name + ".json");
                            layout = new AprilTagFieldLayout(p);
                            layoutString = new ObjectMapper().writeValueAsString(layout);
                        } catch (IOException e) {
                            throw new RuntimeException(e);
                        }
                    }
                }
            }
            return layout;
        }

        public String getLayoutString() {
            if (layoutString == null) {
                getLayout();
            }
            return layoutString;
        }
    }

    /**
     * Officially defined and relevant vertical lines found on the field (defined by X-axis offset)
     */
    public static class LinesVertical {
        public static final double center = fieldLength / 2.0;
        public static final double starting =
                AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(26).get().getX();
        public static final double allianceZone = starting;
        public static final double hubCenter =
                AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(26).get().getX()
                        + Hub.width / 2.0;
        public static final double neutralZoneNear = center - Units.inchesToMeters(120);
        public static final double neutralZoneFar = center + Units.inchesToMeters(120);
        public static final double oppHubCenter =
                AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(4).get().getX()
                        + Hub.width / 2.0;
        public static final double oppAllianceZone =
                AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(10).get().getX();
    }

    /**
     * Officially defined and relevant horizontal lines found on the field (defined by Y-axis
     * offset)
     *
     * <p>NOTE: The field element start and end are always left to right from the perspective of the
     * alliance station
     */
    public static class LinesHorizontal {

        public static final double center = fieldWidth / 2.0;

        // Right of hub
        public static final double rightBumpStart = Hub.nearRightCorner.getY();
        public static final double rightBumpEnd = rightBumpStart - RightBump.width;
        public static final double rightTrenchOpenStart = rightBumpEnd - Units.inchesToMeters(12.0);
        public static final double rightTrenchOpenEnd = 0;

        // Left of hub
        public static final double leftBumpEnd = Hub.nearLeftCorner.getY();
        public static final double leftBumpStart = leftBumpEnd + LeftBump.width;
        public static final double leftTrenchOpenEnd = leftBumpStart + Units.inchesToMeters(12.0);
        public static final double leftTrenchOpenStart = fieldWidth;
    }

    /** Hub related constants */
    public static class Hub {

        // Dimensions
        public static final double width = Units.inchesToMeters(47.0);
        public static final double height =
                Units.inchesToMeters(72.0); // includes the catcher at the top
        public static final double innerWidth = Units.inchesToMeters(41.7);
        public static final double innerHeight = Units.inchesToMeters(56.5);

        // Relevant reference points on alliance side
        public static final Translation3d topCenterPoint =
                new Translation3d(
                        AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(26).get().getX()
                                + width / 2.0,
                        fieldWidth / 2.0,
                        height);
        public static final Translation2d nearLeftCorner =
                new Translation2d(
                        topCenterPoint.getX() - width / 2.0, fieldWidth / 2.0 + width / 2.0);
        public static final Translation2d nearRightCorner =
                new Translation2d(
                        topCenterPoint.getX() - width / 2.0, fieldWidth / 2.0 - width / 2.0);
        public static final Translation2d farLeftCorner =
                new Translation2d(
                        topCenterPoint.getX() + width / 2.0, fieldWidth / 2.0 + width / 2.0);
        public static final Translation2d farRightCorner =
                new Translation2d(
                        topCenterPoint.getX() + width / 2.0, fieldWidth / 2.0 - width / 2.0);
        public static final Translation3d innerCenterPoint =
                new Translation3d(
                        AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(26).get().getX()
                                + width / 2.0,
                        fieldWidth / 2.0,
                        innerHeight);
        // Relevant reference points on the opposite side
        public static final Translation3d oppTopCenterPoint =
                new Translation3d(
                        AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(4).get().getX()
                                + width / 2.0,
                        fieldWidth / 2.0,
                        height);
        public static final Translation2d oppNearLeftCorner =
                new Translation2d(
                        oppTopCenterPoint.getX() - width / 2.0, fieldWidth / 2.0 + width / 2.0);
        public static final Translation2d oppNearRightCorner =
                new Translation2d(
                        oppTopCenterPoint.getX() - width / 2.0, fieldWidth / 2.0 - width / 2.0);
        public static final Translation2d oppFarLeftCorner =
                new Translation2d(
                        oppTopCenterPoint.getX() + width / 2.0, fieldWidth / 2.0 + width / 2.0);
        public static final Translation2d oppFarRightCorner =
                new Translation2d(
                        oppTopCenterPoint.getX() + width / 2.0, fieldWidth / 2.0 - width / 2.0);

        // Hub faces
        public static final Pose2d nearFace =
                AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(26).get().toPose2d();
        public static final Pose2d farFace =
                AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(20).get().toPose2d();
        public static final Pose2d rightFace =
                AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(18).get().toPose2d();
        public static final Pose2d leftFace =
                AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(21).get().toPose2d();
    }

    /** Left Bump related constants */
    public static class LeftBump {

        // Dimensions
        public static final double width = Units.inchesToMeters(73.0);
        public static final double height = Units.inchesToMeters(6.513);
        public static final double depth = Units.inchesToMeters(44.4);

        // Relevant reference points on alliance side
        public static final Translation2d nearLeftCorner =
                new Translation2d(LinesVertical.hubCenter - width / 2, Units.inchesToMeters(255));
        public static final Translation2d nearRightCorner = Hub.nearLeftCorner;
        public static final Translation2d farLeftCorner =
                new Translation2d(LinesVertical.hubCenter + width / 2, Units.inchesToMeters(255));
        public static final Translation2d farRightCorner = Hub.farLeftCorner;

        // Relevant reference points on opposing side
        public static final Translation2d oppNearLeftCorner =
                new Translation2d(LinesVertical.hubCenter - width / 2, Units.inchesToMeters(255));
        public static final Translation2d oppNearRightCorner = Hub.oppNearLeftCorner;
        public static final Translation2d oppFarLeftCorner =
                new Translation2d(LinesVertical.hubCenter + width / 2, Units.inchesToMeters(255));
        public static final Translation2d oppFarRightCorner = Hub.oppFarLeftCorner;
    }

    /** Right Bump related constants */
    public static class RightBump {
        // Dimensions
        public static final double width = Units.inchesToMeters(73.0);
        public static final double height = Units.inchesToMeters(6.513);
        public static final double depth = Units.inchesToMeters(44.4);

        // Relevant reference points on alliance side
        public static final Translation2d nearLeftCorner =
                new Translation2d(LinesVertical.hubCenter + width / 2, Units.inchesToMeters(255));
        public static final Translation2d nearRightCorner = Hub.nearLeftCorner;
        public static final Translation2d farLeftCorner =
                new Translation2d(LinesVertical.hubCenter - width / 2, Units.inchesToMeters(255));
        public static final Translation2d farRightCorner = Hub.farLeftCorner;

        // Relevant reference points on opposing side
        public static final Translation2d oppNearLeftCorner =
                new Translation2d(LinesVertical.hubCenter + width / 2, Units.inchesToMeters(255));
        public static final Translation2d oppNearRightCorner = Hub.oppNearLeftCorner;
        public static final Translation2d oppFarLeftCorner =
                new Translation2d(LinesVertical.hubCenter - width / 2, Units.inchesToMeters(255));
        public static final Translation2d oppFarRightCorner = Hub.oppFarLeftCorner;
    }

    /** Left Trench related constants */
    public static class LeftTrench {
        // Dimensions
        public static final double width = Units.inchesToMeters(65.65);
        public static final double depth = Units.inchesToMeters(47.0);
        public static final double height = Units.inchesToMeters(40.25);
        public static final double openingWidth = Units.inchesToMeters(50.34);
        public static final double openingHeight = Units.inchesToMeters(22.25);

        // Relevant reference points on alliance side
        public static final Translation3d openingTopLeft =
                new Translation3d(LinesVertical.hubCenter, fieldWidth, openingHeight);
        public static final Translation3d openingTopRight =
                new Translation3d(
                        LinesVertical.hubCenter, fieldWidth - openingWidth, openingHeight);

        // Relevant reference points on opposing side
        public static final Translation3d oppOpeningTopLeft =
                new Translation3d(LinesVertical.oppHubCenter, fieldWidth, openingHeight);
        public static final Translation3d oppOpeningTopRight =
                new Translation3d(
                        LinesVertical.oppHubCenter, fieldWidth - openingWidth, openingHeight);
    }

    public static class RightTrench {

        // Dimensions
        public static final double width = Units.inchesToMeters(65.65);
        public static final double depth = Units.inchesToMeters(47.0);
        public static final double height = Units.inchesToMeters(40.25);
        public static final double openingWidth = Units.inchesToMeters(50.34);
        public static final double openingHeight = Units.inchesToMeters(22.25);

        // Relevant reference points on alliance side
        public static final Translation3d openingTopLeft =
                new Translation3d(LinesVertical.hubCenter, openingWidth, openingHeight);
        public static final Translation3d openingTopRight =
                new Translation3d(LinesVertical.hubCenter, 0, openingHeight);

        // Relevant reference points on opposing side
        public static final Translation3d oppOpeningTopLeft =
                new Translation3d(LinesVertical.oppHubCenter, openingWidth, openingHeight);
        public static final Translation3d oppOpeningTopRight =
                new Translation3d(LinesVertical.oppHubCenter, 0, openingHeight);
    }

    /** Tower related constants */
    public static class Tower {
        // Dimensions
        public static final double width = Units.inchesToMeters(49.25);
        public static final double depth = Units.inchesToMeters(45.0);
        public static final double height = Units.inchesToMeters(78.25);
        public static final double innerOpeningWidth = Units.inchesToMeters(32.250);
        public static final double frontFaceX = Units.inchesToMeters(43.51);

        public static final double uprightHeight = Units.inchesToMeters(72.1);

        // Rung heights from the floor
        public static final double lowRungHeight = Units.inchesToMeters(27.0);
        public static final double midRungHeight = Units.inchesToMeters(45.0);
        public static final double highRungHeight = Units.inchesToMeters(63.0);

        // Relevant reference points on alliance side
        public static final Translation2d centerPoint =
                new Translation2d(
                        frontFaceX,
                        AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(31).get().getY());
        public static final Translation2d leftUpright =
                new Translation2d(
                        frontFaceX,
                        (AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(31).get().getY())
                                + innerOpeningWidth / 2
                                + Units.inchesToMeters(0.75));
        public static final Translation2d rightUpright =
                new Translation2d(
                        frontFaceX,
                        (AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(31).get().getY())
                                - innerOpeningWidth / 2
                                - Units.inchesToMeters(0.75));

        // Relevant reference points on opposing side
        public static final Translation2d oppCenterPoint =
                new Translation2d(
                        fieldLength - frontFaceX,
                        AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(15).get().getY());
        public static final Translation2d oppLeftUpright =
                new Translation2d(
                        fieldLength - frontFaceX,
                        (AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(15).get().getY())
                                + innerOpeningWidth / 2
                                + Units.inchesToMeters(0.75));
        public static final Translation2d oppRightUpright =
                new Translation2d(
                        fieldLength - frontFaceX,
                        (AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(15).get().getY())
                                - innerOpeningWidth / 2
                                - Units.inchesToMeters(0.75));
    }

    public static class Depot {
        // Dimensions
        public static final double width = Units.inchesToMeters(42.0);
        public static final double depth = Units.inchesToMeters(27.0);
        public static final double height = Units.inchesToMeters(1.125);
        public static final double distanceFromCenterY = Units.inchesToMeters(75.93);

        // Relevant reference points on alliance side
        public static final Translation3d depotCenter =
                new Translation3d(depth, (fieldWidth / 2) + distanceFromCenterY, height);
        public static final Translation3d leftCorner =
                new Translation3d(
                        depth, (fieldWidth / 2) + distanceFromCenterY + (width / 2), height);
        public static final Translation3d rightCorner =
                new Translation3d(
                        depth, (fieldWidth / 2) + distanceFromCenterY - (width / 2), height);
    }

    public static class Outpost {
        // Dimensions
        public static final double width = Units.inchesToMeters(31.8);
        public static final double openingDistanceFromFloor = Units.inchesToMeters(28.1);
        public static final double height = Units.inchesToMeters(7.0);

        // Relevant reference points on alliance side
        public static final Translation2d centerPoint =
                new Translation2d(
                        0, AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(29).get().getY());
    }
}
```

src/main/frc.robot/Robot.java:

```
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.FollowPathCommand;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.lang.reflect.Field;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
    Timer m_gcTimer = new Timer();
    private Command autonomousCommand;
    private RobotContainer robotContainer;

    public Robot() {
        super(RobotConstants.LOOPER_DT);
        m_gcTimer.start();
    }

    @Override
    public void robotInit() {
        // logger initialization
        // if (Robot.isSimulation()) {
        Logger.addDataReceiver(new NT4Publisher());
        // } // REMOVE before comp
        Logger.addDataReceiver(new WPILOGWriter());

        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.start();

        // config watchdog
        try {
            Field watchdogField = IterativeRobotBase.class.getDeclaredField("m_watchdog");
            watchdogField.setAccessible(true);
            Watchdog watchdog = (Watchdog) watchdogField.get(this);
            watchdog.setTimeout(0.2);
        } catch (Exception e) {
            DriverStation.reportWarning("Failed to disable loop overrun warnings.", false);
        }
        CommandScheduler.getInstance().setPeriod(0.2);

        robotContainer = new RobotContainer();
        robotContainer.setThrottle(false);

        // elastic
        WebServer.start(5800, Filesystem.getDeployDirectory().getPath());

        // warm-up path-following
        CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
    }

    @Override
    public void robotPeriodic() {
        robotContainer.robotPeriodic();
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
        robotContainer.setThrottle(false);
    }

    @Override
    public void autonomousInit() {
        autonomousCommand = robotContainer.getAutonomousCommand();
        robotContainer.setThrottle(true);
        if (autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(autonomousCommand);
        }
    }

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        robotContainer.setThrottle(true);
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        robotContainer.setThrottle(true);
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}
}

```
src/main/frc.robot/RobotConstants.java:

```
package frc.robot;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.wpilibj.RobotController;
import lib.ironpulse.utils.Logging;

/**
 * Robot-wide constants that are used across multiple subsystems. Constants that need tuning are
 * using NTParameter annotation.
 */
public final class RobotConstants {
    public static final String RIOSerial10541 = "0334EE73";
    // Robot timing constants
    public static final double LOOPER_DT = 0.02; // 50Hz control loop
    public static final String ROBORIO_CAN_BUS_NAME = "rio";
    // Hardware device IDs
    public static final int PIGEON_ID = 14; // Pigeon2 IMU device ID
    public static final int LED_PORT = 0;
    public static final int LED_LENGTH = 40;
    public static final CANBus ROBORIO_CAN_BUS = new CANBus(ROBORIO_CAN_BUS_NAME);
    public static boolean is10541 = RIOSerial10541.equals(RobotController.getSerialNumber());
    // CAN bus configuration
    public static final String CANIVORE_CAN_BUS_NAME = is10541 ? "6941Canivore0" : "6941Canivore0";
    public static final CANBus CANIVORE_CAN_BUS = new CANBus(CANIVORE_CAN_BUS_NAME);
    public static boolean disableHAL = false;

    // auto robot config
    public static RobotConfig AUTO_ROBOT_CONFIG;

    static {
        try {
            AUTO_ROBOT_CONFIG = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            Logging.error("Constants", "Failed to load AUTO_ROBOT_CONFIG. %s", e.getMessage());
        }
    }

    private RobotConstants() {
        // Prevent instantiation
    }
}

```

src/main/frc.robot/RobotContainer.java:

```
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Configs.IntakeConfig;
import frc.robot.subsystems.Configs.IntakerExtensionParamsNT;
import frc.robot.subsystems.Configs.IntakerRollerParamsNT;
import frc.robot.subsystems.Configs.SwerveMK5Config;
import frc.robot.subsystems.IntakerSubsystem;
import lib.ironpulse.io.MotorIO;
import lib.ironpulse.io.MotorIOSim;
import lib.ironpulse.io.MotorIOTalonFX;
import lib.ironpulse.io.MotorInputsAutoLogged;
import lib.ironpulse.subsystem.position.PositionMotorSubsystem;
import lib.ironpulse.subsystem.velocity.VelocityMotorSubsystem;
import lib.ironpulse.swerve.Swerve;
import lib.ironpulse.swerve.SwerveCommands;
import lib.ironpulse.swerve.mk5n.ImuIOPigeon;
import lib.ironpulse.swerve.mk5n.SwerveModuleIOMK5N;
import lib.ironpulse.swerve.sim.ImuIOSim;
import lib.ironpulse.swerve.sim.SwerveModuleIOSimpleSim;

public class RobotContainer {
    private static final boolean HAS_INTAKER_ROLLER_IO = true;
    private static final boolean HAS_INTAKER_EXTENSION_IO = true;
    private static final boolean HAS_SWERVE_IO = true;

    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);

    private final Swerve swerve;
    private final IntakerSubsystem intake;

    public RobotContainer() {
        final boolean isReal = RobotBase.isReal();

        swerve = buildSwerve(isReal && HAS_SWERVE_IO);

        var intakerRoller = buildIntakerRoller(isReal && HAS_INTAKER_ROLLER_IO);
        var intakerExtension = buildIntakerExtension(isReal && HAS_INTAKER_EXTENSION_IO);

        intake = new IntakerSubsystem(intakerRoller, intakerExtension);

        // set default commands
        swerve.setDefaultCommand(
                SwerveCommands.driveWithJoystick(
                        swerve,
                        () -> -driver.getLeftY(),
                        () -> -driver.getLeftX(),
                        () -> -driver.getRightX(),
                        () -> null,
                        MetersPerSecond.of(0.04),
                        DegreesPerSecond.of(3.0)));

        intake.setDefaultCommand();

        // basic bindings for intake
        driver.leftBumper().onTrue(intake.toggleIntake());
        driver.leftTrigger().whileTrue(intake.runFeed());
        driver.povDown().onTrue(intake.runRetract());
        operator.leftBumper().whileTrue(intake.runExtendedReverse());
    }

    private Swerve buildSwerve(boolean isReal) {
        return new Swerve(
                isReal ? SwerveMK5Config.kRealConfig : SwerveMK5Config.kSimConfig,
                isReal
                        ? new ImuIOPigeon(SwerveMK5Config.kRealConfig, SwerveMK5Config.pigeonConfig)
                        : new ImuIOSim(),
                isReal
                        ? new SwerveModuleIOMK5N(SwerveMK5Config.kRealConfig, 0)
                        : new SwerveModuleIOSimpleSim(SwerveMK5Config.kSimConfig, 0),
                isReal
                        ? new SwerveModuleIOMK5N(SwerveMK5Config.kRealConfig, 1)
                        : new SwerveModuleIOSimpleSim(SwerveMK5Config.kSimConfig, 1),
                isReal
                        ? new SwerveModuleIOMK5N(SwerveMK5Config.kRealConfig, 2)
                        : new SwerveModuleIOSimpleSim(SwerveMK5Config.kSimConfig, 2),
                isReal
                        ? new SwerveModuleIOMK5N(SwerveMK5Config.kRealConfig, 3)
                        : new SwerveModuleIOSimpleSim(SwerveMK5Config.kSimConfig, 3));
    }

    private VelocityMotorSubsystem<MotorInputsAutoLogged, MotorIO> buildIntakerRoller(
            boolean isReal) {
        return new VelocityMotorSubsystem<>(
                IntakeConfig.INTAKER_ROLLER_CONFIG,
                new MotorInputsAutoLogged(),
                isReal
                        ? new MotorIOTalonFX(IntakeConfig.INTAKER_ROLLER_CONFIG)
                        : new MotorIOSim(IntakeConfig.INTAKER_ROLLER_CONFIG),
                IntakerRollerParamsNT.asVelocityParamSources());
    }

    private PositionMotorSubsystem<
                    MotorInputsAutoLogged, MotorIO, edu.wpi.first.units.measure.Distance>
            buildIntakerExtension(boolean isReal) {
        return new PositionMotorSubsystem<>(
                IntakeConfig.INTAKER_EXTENSION_CONFIG,
                new MotorInputsAutoLogged(),
                isReal
                        ? new MotorIOTalonFX(IntakeConfig.INTAKER_EXTENSION_CONFIG)
                        : new MotorIOSim(IntakeConfig.INTAKER_EXTENSION_CONFIG),
                IntakerExtensionParamsNT.asPositionParamSources(),
                Meters.of(0),
                IntakeConfig.INTAKE_EXTENSION_METERS_PER_ROTATION);
    }

    public void robotPeriodic() {
        // Update NTParameters for Intaker/Swerve subsystems
        lib.ntext.NTParameterRegistry.refresh();
    }

    public void setThrottle(boolean enabled) {
        // Throttle management for subsystems; placeholder for future expansion
    }

    public Command getAutonomousCommand() {
        return null;
    }
}

```

src/main/frc.robot/Main.java:

```
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

public final class Main {
    private Main() {}

    public static void main(String... args) {
        RobotBase.startRobot(Robot::new);
    }
}

```

src/main/frc.robot/RobotStateRecorder.java:

```
package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import lib.ironpulse.math.rbd.TransformRecorder;
import org.littletonrobotics.junction.Logger;

/**
 * RobotStateRecorder for tracking robot pose and transforms. Simplified for Intaker+Swerve only.
 */
public class RobotStateRecorder extends TransformRecorder {
    public static final String kFrameShot = "Shot";
    public static final Translation3d kRobotToShot =
            new Translation3d(Meters.of(0), Meters.of(0.0), Meters.of(0.35));
    private static RobotStateRecorder instance;

    private RobotStateRecorder() {
        setBufferDuration(2.0);
        // add default transforms
        putTransform(
                kTransformWorldDriverStationBlue,
                kFrameWorld,
                kFrameDriverStationBlue); // static: TWorldDSB
        putTransform(
                kTransformWorldDriverStationRed,
                kFrameWorld,
                kFrameDriverStationRed); // static TWorldDSR
        putTransform(
                new Pose3d(),
                Seconds.of(0.0),
                kFrameWorld,
                kFrameRobot); // dynamic TWorldRobot at origin
        putTransform(
                new Pose3d(kRobotToShot, Rotation3d.kZero),
                Seconds.of(0.0),
                kFrameRobot,
                kFrameShot); // dynamic TRobotShot
    }

    public static RobotStateRecorder getInstance() {
        if (instance == null) {
            instance = new RobotStateRecorder();
        }
        return instance;
    }

    public static void periodic() {
        // logging robot pose for swerve
        Logger.recordOutput(
                "RobotStateRecorder/poseWorldRobot", RobotStateRecorder.getPoseWorldRobotCurrent());
        Logger.recordOutput(
                "RobotStateRecorder/RobotRotation2d",
                RobotStateRecorder.getPoseWorldRobotCurrent().getRotation().toRotation2d());
    }

    public static Pose3d getPoseWorldRobotCurrent() {
        return RobotStateRecorder.getInstance()
                .getTransform(
                        Seconds.of(Timer.getTimestamp()),
                        TransformRecorder.kFrameWorld,
                        TransformRecorder.kFrameRobot)
                .orElse(new Pose3d());
    }

    public static Pose3d getPoseWorldShotCurrent() {
        return RobotStateRecorder.getInstance()
                .getTransform(
                        Seconds.of(Timer.getTimestamp()),
                        TransformRecorder.kFrameWorld,
                        RobotStateRecorder.kFrameShot)
                .orElse(new Pose3d());
    }
}

// keep placeholders for potential future integration.

@Getter
@Setter
@AutoLogOutput(key = "RobotStateRecorder/kFrameTarget")
private static String kFrameTarget = kFrameGoal;

private RobotStateRecorder() {
    setBufferDuration(2.0);
    velocityRobotBuffer = TimeInterpolatableBuffer.createBuffer(2.0);
    velocityRobotCmdBuffer = TimeInterpolatableBuffer.createBuffer(2.0);
    // add default transforms
    putTransform(
            kTransformWorldDriverStationBlue,
            kFrameWorld,
            kFrameDriverStationBlue); // static: TWorldDSB
    putTransform(
            kTransformWorldDriverStationRed,
            kFrameWorld,
            kFrameDriverStationRed); // static TWorldDSR
    putTransform(
            new Pose3d(),
            Seconds.of(0.0),
            kFrameWorld,
            kFrameRobot); // dynamic TWorldRobot at origin
    putTransform(
            new Pose3d(kRobotToShot, Rotation3d.kZero),
            Seconds.of(0.0),
            kFrameRobot,
            kFrameShot); // dynamic TRobotShot
    putTransform(
            new Pose3d(FieldConstants.Hub.innerCenterPoint, Rotation3d.kZero),
            Seconds.of(0.0),
            kFrameWorld,
            kFrameGoal); // static TWorldGoal (blue reference)

    putTransform(
            new Pose3d(1.6, 7, 0, new Rotation3d()),
            Seconds.of(0.0),
            kFrameWorld,
            kFrameFeedUp);
    putTransform(
            new Pose3d(1.6, 1, 0, new Rotation3d()),
            Seconds.of(0.0),
            kFrameWorld,
            kFrameFeedDown);
}

public static RobotStateRecorder getInstance() {
    if (instance == null) {
        instance = new RobotStateRecorder();
    }
    return instance;
}

public static void periodic() {
    // logging
    Logger.recordOutput(
            "RobotStateRecorder/poseWorldRobot", RobotStateRecorder.getPoseWorldRobotCurrent());
    Logger.recordOutput(
            "RobotStateRecorder/velocityRobot", RobotStateRecorder.getVelocityRobotCurrent());
    Logger.recordOutput(
            "RobotStateRecorder/RobotRotation2d",
            RobotStateRecorder.getPoseWorldRobotCurrent().getRotation().toRotation2d());
    Logger.recordOutput(
            "RobotStateRecorder/velocityWorldRobot",
            RobotStateRecorder.getVelocityWorldRobotCurrent());
    Logger.recordOutput(
            "RobotStateRecorder/velocityRobotCmd",
            RobotStateRecorder.getVelocityRobotCmdCurrent());
    Logger.recordOutput(
            "RobotStateRecorder/velocityWorldRobotCmd",
            RobotStateRecorder.getVelocityWorldRobotCmdCurrent());
    Logger.recordOutput(
            "RobotStateRecorder/omegaRobotCurrentRadPerSec",
            RobotStateRecorder.getOmegaRobotCurrent().in(RadiansPerSecond));
    Logger.recordOutput(
            "RobotStateRecorder/TargetPoseWorld", getPoseWorldTargetCurrent(kFrameTarget));
    // Shot-frame logging removed because shooting subsystem is not present.
}

public static final Obstacle2d towerZoneBlue =
        new PolygonObstacle2d(
                new Translation2d(
                        0,
                        FieldConstants.Tower.centerPoint.getY()
                                - FieldConstants.Tower.width / 2.0),
                new Translation2d(
                        FieldConstants.Tower.frontFaceX,
                        FieldConstants.Tower.centerPoint.getY()
                                - FieldConstants.Tower.width / 2.0),
                new Translation2d(
                        FieldConstants.Tower.frontFaceX,
                        FieldConstants.Tower.centerPoint.getY()
                                + FieldConstants.Tower.width / 2.0),
                new Translation2d(
                        0,
                        FieldConstants.Tower.centerPoint.getY()
                                + FieldConstants.Tower.width / 2.0));
public static final Obstacle2d towerZoneRed =
        new PolygonObstacle2d(
                new Translation2d(
                        FieldConstants.fieldLength - FieldConstants.Tower.frontFaceX,
                        FieldConstants.Tower.oppCenterPoint.getY()
                                - FieldConstants.Tower.width / 2.0),
                new Translation2d(
                        FieldConstants.fieldLength,
                        FieldConstants.Tower.oppCenterPoint.getY()
                                - FieldConstants.Tower.width / 2.0),
                new Translation2d(
                        FieldConstants.fieldLength,
                        FieldConstants.Tower.oppCenterPoint.getY()
                                + FieldConstants.Tower.width / 2.0),
                new Translation2d(
                        FieldConstants.fieldLength - FieldConstants.Tower.frontFaceX,
                        FieldConstants.Tower.oppCenterPoint.getY()
                                + FieldConstants.Tower.width / 2.0));

// Hub shooting zones
public static final Obstacle2d hubZoneBlue =
        new PolygonObstacle2d(
                new Translation2d(
                        5.0,
                        FieldConstants.Hub.topCenterPoint.getY()
                                - FieldConstants.Hub.width / 2.0),
                new Translation2d(
                        7.3,
                        FieldConstants.Hub.topCenterPoint.getY()
                                - FieldConstants.Hub.width / 2.0),
                new Translation2d(
                        7.3,
                        FieldConstants.Hub.topCenterPoint.getY()
                                + FieldConstants.Hub.width / 2.0),
                new Translation2d(
                        5.0,
                        FieldConstants.Hub.topCenterPoint.getY()
                                + FieldConstants.Hub.width / 2.0));
public static final Obstacle2d hubZoneRed =
        new PolygonObstacle2d(
                new Translation2d(
                        FieldConstants.fieldLength - 7.3,
                        FieldConstants.Hub.oppTopCenterPoint.getY()
                                - FieldConstants.Hub.width / 2.0),
                new Translation2d(
                        FieldConstants.fieldLength - 5.0,
                        FieldConstants.Hub.oppTopCenterPoint.getY()
                                - FieldConstants.Hub.width / 2.0),
                new Translation2d(
                        FieldConstants.fieldLength - 5.0,
                        FieldConstants.Hub.oppTopCenterPoint.getY()
                                + FieldConstants.Hub.width / 2.0),
                new Translation2d(
                        FieldConstants.fieldLength - 7.3,
                        FieldConstants.Hub.oppTopCenterPoint.getY()
                                + FieldConstants.Hub.width / 2.0));

public static void putVelocityRobot(Time time, ChassisSpeeds speed) {
    velocityRobotBuffer.addSample(time.in(Seconds), toPose2d(speed));
}

public static void putVelocityRobotCmd(Time time, ChassisSpeeds speedCmd) {
    velocityRobotCmdBuffer.addSample(time.in(Seconds), toPose2d(speedCmd));
}

public static void putOmegaRobotCurrent(AngularVelocity omega) {
    omegaRobotCurrent = omega;
}

public static Pose2d getVelocityRobotCurrent() {
    return velocityRobotBuffer.getSample(Timer.getTimestamp()).orElse(new Pose2d());
}

public static AngularVelocity getOmegaRobotCurrent() {
    return omegaRobotCurrent;
}

public static Pose2d getVelocityRobotCmdCurrent() {
    return velocityRobotCmdBuffer.getSample(Timer.getTimestamp()).orElse(new Pose2d());
}

public static Pose2d getVelocityWorldRobotCurrent() {
    // robot-relative velocity (dx, dy, dθ) and current robot pose in world
    Pose2d velocityRobot = getVelocityRobotCurrent();
    Pose3d poseWorldRobot = getPoseWorldRobotCurrent();

    // drop to 2D to get the robot's heading in the XY plane
    Pose2d pose2dWR = poseWorldRobot.toPose2d();
    Translation2d velRobotTrans = velocityRobot.getTranslation();

    // rotate the translational velocity by the robot’s heading
    Translation2d velWorldTrans = velRobotTrans.rotateBy(pose2dWR.getRotation());

    // preserve the same angular component
    return new Pose2d(velWorldTrans, velocityRobot.getRotation());
}

public static Pose2d getVelocityWorldRobotCmdCurrent() {
    // robot-relative command velocity (dx, dy, dθ) and current robot pose in world
    Pose2d velocityRobotCmd = getVelocityRobotCmdCurrent();
    Pose3d poseWorldRobot = getPoseWorldRobotCurrent();

    // drop to 2D to get the robot's heading in the XY plane
    Pose2d pose2dWR = poseWorldRobot.toPose2d();
    Translation2d velRobotCmdTrans = velocityRobotCmd.getTranslation();

    // rotate the translational velocity by the robot’s heading
    Translation2d velWorldCmdTrans = velRobotCmdTrans.rotateBy(pose2dWR.getRotation());

    // preserve the same angular component
    return new Pose2d(velWorldCmdTrans, velocityRobotCmd.getRotation());
}

public static Pose3d getPoseWorldRobotCurrent() {
    return RobotStateRecorder.getInstance()
            .getTransform(
                    Seconds.of(Timer.getTimestamp()),
                    TransformRecorder.kFrameWorld,
                    TransformRecorder.kFrameRobot)
            .orElse(new Pose3d());
}

public static Pose3d getPoseWorldShotCurrent() {
    return RobotStateRecorder.getInstance()
            .getTransform(
                    Seconds.of(Timer.getTimestamp()),
                    TransformRecorder.kFrameWorld,
                    RobotStateRecorder.kFrameShot)
            .orElse(new Pose3d());
}

public static Pose3d getPoseDriverRobotCurrent() {
    return RobotStateRecorder.getInstance()
            .getTransform(
                    Seconds.of(Timer.getTimestamp()),
                    DriverStation.getAlliance()
                            .orElse(DriverStation.Alliance.Blue)
                            .equals(DriverStation.Alliance.Blue)
                            ? RobotStateRecorder.kFrameDriverStationBlue
                            : RobotStateRecorder.kFrameDriverStationRed,
                    TransformRecorder.kFrameRobot)
            .orElse(new Pose3d());
}

public static Pose3d getPoseWorldTargetCurrent(String targetFrame) {
    Pose3d poseBlue =
            RobotStateRecorder.getInstance()
                    .getTransform(
                            Seconds.of(Timer.getTimestamp()),
                            TransformRecorder.kFrameWorld,
                            targetFrame)
                    .orElse(new Pose3d());
    return AllianceFlipUtil.apply(poseBlue);
}

public static Translation2d getTranslationShotToTargetCurrent(String targetFrame) {
    Pose3d shotPoseWorld = getPoseWorldShotCurrent();
    Pose3d targetPoseWorld = getPoseWorldTargetCurrent(targetFrame);
    Translation3d delta =
            targetPoseWorld.getTranslation().minus(shotPoseWorld.getTranslation());
    return delta.toTranslation2d();
}
}

```

