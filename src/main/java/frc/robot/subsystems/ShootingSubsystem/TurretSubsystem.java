package frc.robot.subsystems.ShootingSubsystem;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static frc.robot.subsystems.Configs.TurretConfig.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotStateRecorder;
import frc.robot.subsystems.Configs.TurretConfig;
import frc.robot.subsystems.Configs.TurretPosParamsNT;
import java.util.function.Supplier;
import lib.ironpulse.io.CANCoderIO;
import lib.ironpulse.io.CANCoderIOInputsAutoLogged;
import lib.ironpulse.io.MotorIO;
import lib.ironpulse.io.MotorInputsAutoLogged;
import lib.ironpulse.subsystem.SubsystemConfig;
import lib.ironpulse.subsystem.velocity.VelocityMotorSubsystem;
import lib.ironpulse.subsystem.velocity.VelocityParamSources;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/*
TurretSubsystem
Flow:
systemConstructor
    -> claculate absolute position using differential encoder
    -> sets the position of the embedded encoder ONLY when the syste is constructed
    @see updateUnwrappedTurretAngle
Exposed Commands:
    runTurretPoseWorld : convert the world angle to a robot relative angle and set the position in the system
    runTurretPoseRobot : set the position of the turret
        -> get the shortest target angle
        -> calculate the desired velocity using profiled PID controller
        -> (optional) add the chassis rotation to the desired velocity to compansate for the chassis rotation
        -> sets the desired velocity to the motorio
 */

public class TurretSubsystem extends VelocityMotorSubsystem<MotorInputsAutoLogged, MotorIO> {
    private static final Angle FULL_ROTATION = Degrees.of(360.0);
    private static final double DIFFERENTIAL_SLOPE =
            (TurretConfig.G2_TOOTH_COUNT * TurretConfig.G1_TOOTH_COUNT)
                    / ((TurretConfig.G1_TOOTH_COUNT - TurretConfig.G2_TOOTH_COUNT)
                            * (double) TurretConfig.G0_TOOTH_COUNT);

    private final CANCoderIO encoderG1;
    private final CANCoderIO encoderG2;
    private Angle unwrappedTurretAngle = Degrees.of(0.0);
    private final CANCoderIOInputsAutoLogged encoderG1Inputs = new CANCoderIOInputsAutoLogged();
    private final CANCoderIOInputsAutoLogged encoderG2Inputs = new CANCoderIOInputsAutoLogged();
    private final Alert wrappingAlert =
            new Alert("Turret unwrapping difference is huge", Alert.AlertType.kError);
    private final Alert invalidHysteresisAlert =
            new Alert(
                    "Turret hysteresis invalid: seekEnterErrorDegrees must be > trackEnterErrorDegrees",
                    Alert.AlertType.kWarning);
    private final ProfiledPIDController posVelCtl =
            new ProfiledPIDController(
                    TurretPosParamsNT.kpSeek.getValue(),
                    TurretPosParamsNT.kiSeek.getValue(),
                    TurretPosParamsNT.kdSeek.getValue(),
                    new TrapezoidProfile.Constraints(
                            TurretPosParamsNT.maxVelocityRPS.getValue() * 360.0,
                            TurretPosParamsNT.maxAccelerationRPS2.getValue() * 360.0));

    @Getter
    @Setter
    @AutoLogOutput(key = "Turret/currentMode")
    private TurretMode currentMode = TurretMode.SEEKING;

    private TurretMode lastControllerMode = null;
    private Supplier<Angle> targetAngleWorld = () -> Degrees.of(0.0);
    private Angle targetAngleRobotWrapped = Degrees.of(0.0);
    private Angle targetAngleRobot = Degrees.of(0.0);

    @AutoLogOutput(key = "Turret/aimErrorAbsDeg")
    private double absAimErrorDeg = 0.0;

    private AngularVelocity desiredTurretVelocity = DegreesPerSecond.of(0.0);

    public TurretSubsystem(
            SubsystemConfig config,
            MotorInputsAutoLogged inputs,
            MotorIO io,
            CANCoderIO encoderG1,
            CANCoderIO encoderG2,
            VelocityParamSources params) {
        super(config, inputs, io, params);
        this.encoderG1 = encoderG1;
        this.encoderG2 = encoderG2;
        updateUnwrappedTurretAngle();
        io.setCurrentPosition(unwrappedTurretAngle);
    }

    @Override
    public void periodic() {
        // updateUnwrappedTurretAngle();
        targetAngleRobotWrapped = toRobotRelativeFromWorld(targetAngleWorld.get());
        targetAngleRobot = unwrapTargetAngle(targetAngleRobotWrapped, getPosition());
        updateModeFromErrorHysteresis();
        updateController(currentMode);
        super.periodic();
        // wrappingAlert.set(!getPosition().isNear(unwrappedTurretAngle, Degrees.of(1.0)));
    }

    @Override
    protected void logState() {
        double encoderDeltaRawDeg =
                (encoderG2Inputs.positionRotations - encoderG1Inputs.positionRotations) * 360.0;
        double encoderDeltaWrappedDeg = encoderDeltaRawDeg;
        if (encoderDeltaWrappedDeg > ENCODER_DELTA_WRAP_THRESHOLD.in(Degrees)) {
            encoderDeltaWrappedDeg -= 360.0;
        } else if (encoderDeltaWrappedDeg < -ENCODER_DELTA_WRAP_THRESHOLD.in(Degrees)) {
            encoderDeltaWrappedDeg += 360.0;
        }
        Logger.processInputs(getName() + "/Absolute/encoderG1", encoderG1Inputs);
        Logger.processInputs(getName() + "/Absolute/encoderG2", encoderG2Inputs);
        Logger.recordOutput(getName() + "/Absolute/encoderDeltaRawDeg", encoderDeltaRawDeg);
        Logger.recordOutput(getName() + "/Absolute/encoderDeltaWrappedDeg", encoderDeltaWrappedDeg);
        Logger.recordOutput(
                getName() + "/Absolute/unwrappedTurretAngle", unwrappedTurretAngle.in(Degrees));
        Logger.recordOutput(getName() + "/targetAngleWorldDeg", targetAngleWorld.get().in(Degrees));
        Logger.recordOutput(getName() + "/targetAngleRobotDeg", targetAngleRobot.in(Degrees));
        Logger.recordOutput(getName() + "/targetVelocity", getCurrSetpoint().in(DegreesPerSecond));
        Logger.recordOutput(getName() + "/atGoal/position", positionAtGoal());
        Logger.recordOutput(getName() + "/atGoal/velocity", velocityAtGoal());
        Logger.recordOutput(getName() + "/atGoal", atGoal());
        Logger.recordOutput(getName() + "/currAngleRobotDeg", getPosition().in(Degrees));
        Logger.recordOutput(getName() + "/currVelocity", getVelocity().in(DegreesPerSecond));
    }

    public Command setTurretPoseWorld(Supplier<Angle> targetAngleSupplier) {
        return Commands.runOnce(
                () -> {
                    targetAngleWorld = targetAngleSupplier;
                });
    }

    public Command runTurretTargetLoop() {
        return Commands.runOnce(() -> posVelCtl.reset(getPosition().in(Degrees)))
                .andThen(runVelVolt(() -> calculateTargetVelocity(targetAngleRobot)));
    }

    private void updateController(TurretMode mode) {
        if (mode != lastControllerMode || TurretPosParamsNT.isAnyChanged()) {

            switch (mode) {
                case TRACKING:
                    posVelCtl.setP(TurretPosParamsNT.kpTrack.getValue());
                    posVelCtl.setI(TurretPosParamsNT.kiTrack.getValue());
                    posVelCtl.setD(TurretPosParamsNT.kdTrack.getValue());
                    posVelCtl.setConstraints(new TrapezoidProfile.Constraints(1.0e6, 1.0e6));
                    break;
                case SEEKING:
                    posVelCtl.setP(TurretPosParamsNT.kpSeek.getValue());
                    posVelCtl.setI(TurretPosParamsNT.kiSeek.getValue());
                    posVelCtl.setD(TurretPosParamsNT.kdSeek.getValue());
                    posVelCtl.setConstraints(
                            new TrapezoidProfile.Constraints(
                                    TurretPosParamsNT.maxVelocityRPS.getValue() * 360.0,
                                    TurretPosParamsNT.maxAccelerationRPS2.getValue() * 360.0));
                    break;
            }
        }
        lastControllerMode = mode;
    }

    private AngularVelocity calculateTargetVelocity(Angle targetAngle) {

        double desiredVelocity =
                posVelCtl.calculate(getPosition().in(Degrees), targetAngle.in(Degrees));
        // compansate for the chassis rotation
        double chassisOmegaDegPerSec =
                RobotStateRecorder.getVelocityWorldRobotCurrent().getRotation().getDegrees();
        // Logger.recordOutput(getName() + "/addedV", chassisOmegaDegPerSec);
        desiredVelocity -=
                TurretPosParamsNT.kchassisVelCompensation.getValue() * chassisOmegaDegPerSec;
        desiredTurretVelocity = DegreesPerSecond.of(desiredVelocity);
        return desiredTurretVelocity;
    }

    public boolean positionAtGoal() {
        Angle currentAngle = getPosition();
        return currentAngle.isNear(
                targetAngleRobot,
                Degrees.of(TurretPosParamsNT.positionAtGoalToleranceDegrees.getValue()));
    }

    public boolean atGoal() {
        return positionAtGoal() && velocityAtGoal();
    }

    // MODE switching Logic
    private void updateModeFromErrorHysteresis() {
        double seekEnterDeg = TurretConfig.TurretPosParams.seekEnterErrorDegrees;
        double trackEnterDeg = TurretConfig.TurretPosParams.trackEnterErrorDegrees;
        boolean invalidThresholds = seekEnterDeg <= trackEnterDeg;
        invalidHysteresisAlert.set(invalidThresholds);
        if (invalidThresholds) {
            seekEnterDeg = 1.2;
            trackEnterDeg = 0.8;
        }

        absAimErrorDeg = Math.abs(targetAngleRobot.minus(getPosition()).in(Degrees));
        if (absAimErrorDeg > seekEnterDeg) {
            currentMode = TurretMode.SEEKING;
        } else if (absAimErrorDeg < trackEnterDeg) {
            currentMode = TurretMode.TRACKING;
        }
    }

    // CMD target unwrapping logic
    private Angle unwrapTargetAngle(Angle targetAngleWrapped, Angle currentAngleUnwrapped) {
        double currentContinuous = currentAngleUnwrapped.in(Degrees);
        double targetPosition = targetAngleWrapped.in(Degrees);
        double currentWrapped = Rotation2d.fromDegrees(currentContinuous).getDegrees();
        double closestOffset = targetPosition - currentWrapped;
        if (closestOffset > FULL_ROTATION.in(Degrees) / 2.0) {
            closestOffset -= FULL_ROTATION.in(Degrees);
        } else if (closestOffset < -FULL_ROTATION.in(Degrees) / 2.0) {
            closestOffset += FULL_ROTATION.in(Degrees);
        }

        double finalOffset = currentContinuous + closestOffset;
        if (MathUtil.inputModulus(currentContinuous + closestOffset, 0.0, FULL_ROTATION.in(Degrees))
                == MathUtil.inputModulus(
                        currentContinuous - closestOffset, 0.0, FULL_ROTATION.in(Degrees))) {
            if (finalOffset > 0.0) {
                finalOffset = currentContinuous - Math.abs(closestOffset);
            } else {
                finalOffset = currentContinuous + Math.abs(closestOffset);
            }
        }

        double forwardLimit =
                TURRET_SOFT_LIMIT_CCW.in(Degrees) - TURRET_SOFT_LIMIT_MARGIN.in(Degrees);
        double reverseLimit =
                TURRET_SOFT_LIMIT_CW.in(Degrees) + TURRET_SOFT_LIMIT_MARGIN.in(Degrees);
        if (!Double.isNaN(forwardLimit) && finalOffset > forwardLimit) {
            finalOffset -= FULL_ROTATION.in(Degrees);
        } else if (!Double.isNaN(reverseLimit) && finalOffset < reverseLimit) {
            finalOffset += FULL_ROTATION.in(Degrees);
        }

        return Degrees.of(finalOffset);
    }

    private Angle toRobotRelativeFromWorld(Angle worldAngle) {
        Rotation2d worldRotation = Rotation2d.fromDegrees(worldAngle.in(Degrees));
        Rotation2d robotRotation =
                RobotStateRecorder.getPoseWorldRobotCurrent().toPose2d().getRotation();
        Rotation2d robotRelative = worldRotation.rotateBy(robotRotation.unaryMinus());
        return Degrees.of(robotRelative.getDegrees());
    }

    // absolute encoder angle unwrapping logic, only used currently for starting position
    private void updateUnwrappedTurretAngle() {
        encoderG1.readInputs(encoderG1Inputs);
        encoderG2.readInputs(encoderG2Inputs);

        Angle encoderG1Angle = Degrees.of(encoderG1Inputs.positionRotations * 360.0);
        Angle encoderG2Angle = Degrees.of(encoderG2Inputs.positionRotations * 360.0);
        unwrappedTurretAngle = unwrapDifferentialAngle(encoderG1Angle, encoderG2Angle);
    }

    private static Angle unwrapDifferentialAngle(Angle encoderGearAAngle, Angle encoderGearBAngle) {
        Angle encoderDelta = encoderGearBAngle.minus(encoderGearAAngle);
        if (encoderDelta.gt(ENCODER_DELTA_WRAP_THRESHOLD)) {
            encoderDelta = encoderDelta.minus(Degrees.of(360.0));
        } else if (encoderDelta.lt(ENCODER_DELTA_WRAP_THRESHOLD.unaryMinus())) {
            encoderDelta = encoderDelta.plus(Degrees.of(360.0));
        }

        Angle turretAngleFromDelta = encoderDelta.times(DIFFERENTIAL_SLOPE);
        double encoderARotationsEstimate =
                (turretAngleFromDelta.in(Degrees) * G0_TOOTH_COUNT / G1_TOOTH_COUNT) / 360.0;
        double encoderARotationsFloor = Math.floor(encoderARotationsEstimate);
        Angle turretAngle =
                Degrees.of(encoderARotationsFloor * 360.0)
                        .plus(encoderGearAAngle)
                        .times(G1_TOOTH_COUNT / (double) G0_TOOTH_COUNT);
        Angle correctionError = turretAngle.minus(turretAngleFromDelta);
        if (correctionError.lt(ANGLE_CORRECTION_THRESHOLD.unaryMinus())) {
            turretAngle =
                    turretAngle.plus(
                            Degrees.of((G1_TOOTH_COUNT / (double) G0_TOOTH_COUNT) * 360.0));
        } else if (correctionError.gt(ANGLE_CORRECTION_THRESHOLD)) {
            turretAngle =
                    turretAngle.minus(
                            Degrees.of((G1_TOOTH_COUNT / (double) G0_TOOTH_COUNT) * 360.0));
        }

        return turretAngle;
    }

    public Command setCurrentPosition(Angle pos) {
        return Commands.runOnce(() -> io.setCurrentPosition(pos));
    }

    public enum TurretMode {
        TRACKING,
        SEEKING
    }
}
