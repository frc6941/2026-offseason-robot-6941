package frc.robot.subsystems.ShootingSubsystem;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.subsystems.Configs.TurretConfig.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
import frc.robot.subsystems.Configs.TurretConfig;
import lib.ironpulse.io.CANCoderIO;
import lib.ironpulse.io.CANCoderIOInputsAutoLogged;
import lib.ironpulse.io.MotorIO;
import lib.ironpulse.io.MotorInputsAutoLogged;
import lib.ironpulse.subsystem.SubsystemConfig;
import lib.ironpulse.subsystem.velocity.VelocityMotorSubsystem;
import lib.ironpulse.subsystem.velocity.VelocityParamSources;
import org.littletonrobotics.junction.Logger;

public class TurretSubsystem extends VelocityMotorSubsystem<MotorInputsAutoLogged, MotorIO> {
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
        super.periodic();
        updateUnwrappedTurretAngle();
        wrappingAlert.set(!getPosition().isNear(unwrappedTurretAngle, Degrees.of(1.0)));
    }

    private void updateUnwrappedTurretAngle() {
        encoderG1.readInputs(encoderG1Inputs);
        encoderG2.readInputs(encoderG2Inputs);
        Logger.processInputs("Subsystem/" + getName() + "/encoderG1", encoderG1Inputs);
        Logger.processInputs("Subsystem/" + getName() + "/encoderG2", encoderG2Inputs);
        Angle encoderG1Angle = Degrees.of(encoderG1Inputs.positionRotations * 360.0);
        Angle encoderG2Angle = Degrees.of(encoderG2Inputs.positionRotations * 360.0);
        unwrappedTurretAngle = unwrapDifferentialAngle(encoderG1Angle, encoderG2Angle);
        Logger.recordOutput(
                "Subsystem/" + getName() + "/unwrappedTurretAngle",
                unwrappedTurretAngle.in(Degrees));
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
}
