package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import lib.ironpulse.io.MotorIO;
import lib.ironpulse.io.MotorIOSim;
import lib.ironpulse.io.MotorIOTalonFX;
import lib.ironpulse.io.MotorInputsAutoLogged;
import lib.ironpulse.subsystem.position.PositionMotorSubsystem;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * Elevator mechanism using ServoMotorSubsystem with custom functionality for zeroing and
 * characterization.
 */
public class ElevatorSubsystem
        extends PositionMotorSubsystem<MotorInputsAutoLogged, MotorIO, Distance> {

    @Getter double setPtMeter = 0.0;

    @Getter
    @AutoLogOutput(key = "Elevator/isGoingUp")
    private boolean isGoingUp = false;

    private double previousSetPtMeter = 0;

    public ElevatorSubsystem() {
        super(
                ElevatorConfig.CONFIG,
                new MotorInputsAutoLogged(),
                createIO(),
                ElevatorParamsNT.asPositionParamSources(),
                Meters.of(0),
                Meters.of(ElevatorConfig.METERS_PER_ROTATION));
    }

    private static MotorIO createIO() {
        if (Logger.hasReplaySource()) {
            return new MotorIO() {};
        } else if (RobotBase.isReal()) {
            return new MotorIOTalonFX(ElevatorConfig.CONFIG);
        } else {
            return new MotorIOSim(ElevatorConfig.CONFIG);
        }
    }

    @Override
    public void periodic() {
        super.periodic();

        setPtMeter = getCurrSetpoint().in(Meters);
        if (setPtMeter != previousSetPtMeter) {
            isGoingUp = setPtMeter > previousSetPtMeter;
            previousSetPtMeter = setPtMeter;
        }
    }

    @Override
    public Command runMotionMagic(Distance meters) {
        return meters.in(Meters) > getCurrPos().in(Meters)
                ? super.runMotionMagic( // going up
                        meters,
                        ElevatorParamsNT.motionMagicVelRPSUp.getValue(),
                        ElevatorParamsNT.motionMagicAccelRPS2Up.getValue(),
                        ElevatorParamsNT.motionMagicJerkRPS3Up.getValue())
                : super.runMotionMagic( // going down
                        meters,
                        ElevatorParamsNT.motionMagicVelRPSDown.getValue(),
                        ElevatorParamsNT.motionMagicAccelRPS2Down.getValue(),
                        ElevatorParamsNT.motionMagicJerkRPS3Down.getValue());
    }

    public Command setElevatorPosition(Distance meters) {
        return runMotionMagic(meters);
    }
}
