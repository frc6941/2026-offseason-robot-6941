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

/**
 * Elevator mechanism using ServoMotorSubsystem with custom functionality for
 * zeroing and characterization.
 */
public class ElevatorSubsystem extends ServoMotorSubsystem<MotorInputsAutoLogged, MotorIO, Distance> {

    @Getter
    double setPtMeter = 0.0;

    @Getter
    private boolean atGoal = false;

    @Getter
    @AutoLogOutput(key = "Elevator/isGoingUp")
    private boolean isGoingUp = false;

    private double previousSetPtMeter = 0.16;

    public ElevatorSubsystem() {
        super(
                ElevatorConfig.CONFIG,
                new MotorInputsAutoLogged(),
                createIO(),
                ElevatorParamsNT.asServoMotorParamSources(),
                Meters.of(0),
                Meters.of(ElevatorConfig.METERS_PER_ROTATION));
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
        setPtMeter = getCurrSetpoint().getSetPoint().in(Meters);
        if (setPtMeter != previousSetPtMeter) {
            isGoingUp = setPtMeter > previousSetPtMeter;
            previousSetPtMeter = setPtMeter;
        }

    }


    @Override
    public void setMotionMagicSetpoint(Distance meters) {
        boolean goingUp = meters.in(Meters) > getCurrPos().in(Meters);
        if (goingUp) {
            super.setMotionMagicSetpoint(
                    meters,
                    ElevatorParamsNT.motionMagicVelRPSUp.getValue(),
                    ElevatorParamsNT.motionMagicAccelRPS2Up.getValue(),
                    ElevatorParamsNT.motionMagicJerkRPS3Up.getValue());
        } else {
            super.setMotionMagicSetpoint(
                    meters,
                    ElevatorParamsNT.motionMagicVelRPSDown.getValue(),
                    ElevatorParamsNT.motionMagicAccelRPS2Down.getValue(),
                    ElevatorParamsNT.motionMagicJerkRPS3Down.getValue());
        }
    }

    public void setElevatorPosition(Distance meters) {
        setMotionMagicSetpoint(meters);
    }
}
