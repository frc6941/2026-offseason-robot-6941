package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotBase;
import lib.ironpulse.io.MotorIO;
import lib.ironpulse.io.MotorIOSim;
import lib.ironpulse.io.MotorIOTalonFX;
import lib.ironpulse.io.MotorInputsAutoLogged;
import lib.ironpulse.subsystem.servo.ServoMotorSubsystem;
import lombok.Getter;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

/**
 * Elevator mechanism using ServoMotorSubsystem with custom functionality for
 * zeroing and characterization.
 */
public class ElevatorSubsystem extends ServoMotorSubsystem<MotorInputsAutoLogged, MotorIO> {

    @AutoLogOutput(key = "Elevator/setPoint")
    @Getter
    double wantedPosition = 0.0;

    @Getter
    @AutoLogOutput(key = "Elevator/atGoal")
    private boolean atGoal = false;

    @Getter
    @AutoLogOutput(key = "Elevator/isGoingUp")
    private boolean isGoingUp = false;

    private double previousWantedPosition = 0.16;

    public ElevatorSubsystem() {
        super(
                ElevatorConfig.CONFIG,
                new MotorInputsAutoLogged(),
                createIO(),
                ElevatorParamsNT.asServoMotorParamSources());
    }

    private static MotorIO createIO() {
        if (Logger.hasReplaySource()) {
            return new MotorIO() {
            };
        } else if (RobotBase.isReal()) {
            return new MotorIOTalonFX(ElevatorConfig.CONFIG);
        } else {
            return new MotorIOSim(ElevatorConfig.CONFIG);
        }
    }

    @Override
    public void periodic() {
        super.periodic();

        atGoal = positionAtGoal();
        if (wantedPosition != previousWantedPosition) {
            isGoingUp = wantedPosition > previousWantedPosition;
            previousWantedPosition = wantedPosition;
        }

    }


    public void setElevatorPosition(Distance meters) {
        double currentPos = getServoAngleRot();
        boolean goingUp = Math.abs(meters.in(Meters)) > Math.abs(currentPos);
        this.wantedPosition = meters.in(Meters);
        if (goingUp) {
            setMotionMagicSetpoint(
                    Rotations.of(meters.in(Meters) / ElevatorConfig.METERS_PER_ROTATION),
                    ElevatorParamsNT.motionMagicVelRPSUp.getValue(),
                    ElevatorParamsNT.motionMagicAccelRPS2Up.getValue(),
                    ElevatorParamsNT.motionMagicJerkRPS3Up.getValue());
        } else {
            setMotionMagicSetpoint(
                    Rotations.of(meters.in(Meters) / ElevatorConfig.METERS_PER_ROTATION),
                    ElevatorParamsNT.motionMagicVelRPSDown.getValue(),
                    ElevatorParamsNT.motionMagicAccelRPS2Down.getValue(),
                    ElevatorParamsNT.motionMagicJerkRPS3Down.getValue());
        }
    }

 

}