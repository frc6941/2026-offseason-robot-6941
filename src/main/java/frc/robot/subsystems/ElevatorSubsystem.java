package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj.RobotBase;
import lib.ironpulse.io.MotorIO;
import lib.ironpulse.io.MotorIOSim;
import lib.ironpulse.io.MotorIOTalonFX;
import lib.ironpulse.io.MotorInputsAutoLogged;
import lib.ironpulse.subsystem.servo.ServoMotorSubsystem;
import lib.ironpulse.subsystem.servo.ServoOutputsAutoLogged;
import lombok.Getter;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

/** Elevator mechanism using ServoMotorSubsystem with custom functionality for zeroing and characterization. */
public class ElevatorSubsystem extends ServoMotorSubsystem<MotorInputsAutoLogged, MotorIO, ServoOutputsAutoLogged> {

    @AutoLogOutput(key = "Elevator/setPoint")
    @Getter
    double wantedPosition = 0.0;

    @Getter
    @AutoLogOutput(key = "Elevator/atGoal")
    private boolean atGoal = false;
    
    @Getter
    @AutoLogOutput(key = "Elevator/isGoingUp")
    private boolean isGoingUp = false;

    
    @AutoLogOutput(key = "Elevator/stopDueToLimit")
    private boolean stopDueToLimit = false;
    
    private double previousWantedPosition = 0.16;

    public ElevatorSubsystem() {
        super(
            ElevatorConfig.CONFIG,
            new MotorInputsAutoLogged(),
            createIO(),
            new ServoOutputsAutoLogged(),
            ElevatorParamsNT.asServoMotorParamSources()
        );
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
        
        
        if (wantedPosition > ElevatorParamsNT.maxExtensionMeters.getValue()) {
            stopDueToLimit = true;
        } else if (stopDueToLimit) {
            stopDueToLimit = false;
        }

            atGoal = positionAtGoal(Rotations.of(ElevatorParamsNT.atGoalToleranceMeters.getValue() / ElevatorConfig.CONFIG.metersPerRotation));
            
            if (wantedPosition != previousWantedPosition) {
                isGoingUp = wantedPosition > previousWantedPosition;
                previousWantedPosition = wantedPosition;
            }

    }

    public void setElevatorPosition(DoubleSupplier position) {
        setElevatorPosition(position.getAsDouble());
    }

    public void setElevatorPosition(double meters) {
        double currentPos = getMechanismPositionFromMotor().in(Meters);
        boolean goingUp = meters > currentPos;
        this.wantedPosition = meters;
        if (goingUp) {
            setMotionMagicSetpoint(
                Rotations.of(meters / ElevatorConfig.CONFIG.metersPerRotation),
                ElevatorParamsNT.motionMagicVelRPSUp.getValue(),
                ElevatorParamsNT.motionMagicAccelRPS2Up.getValue(),
                ElevatorParamsNT.motionMagicJerkRPS3Up.getValue()
            );
        } else {
            setMotionMagicSetpoint(
                Rotations.of(meters / ElevatorConfig.CONFIG.metersPerRotation),
                ElevatorParamsNT.motionMagicVelRPSDown.getValue(),
                ElevatorParamsNT.motionMagicAccelRPS2Down.getValue(),
                ElevatorParamsNT.motionMagicJerkRPS3Down.getValue()
            );
        }
    }
    
    public boolean elevatorAtGoal(double offset) {
        return positionAtGoal(Rotations.of(offset / ElevatorConfig.CONFIG.metersPerRotation));
    }

    public void elevatorVoltage(double voltage){
      setVoltage(voltage);
    }


    public boolean isSafeToFlip() {
        return (getMechanismPositionFromMotor().in(Meters) > ElevatorParamsNT.safeHeightFlip.getValue());
    }

}