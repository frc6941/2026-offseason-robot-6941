package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import lib.ironpulse.io.MotorIO;
import lib.ironpulse.io.MotorIOSim;
import lib.ironpulse.io.MotorIOTalonFX;
import lib.ironpulse.io.MotorInputsAutoLogged;
import lib.ironpulse.subsystem.ServoMotorSubsystem;
import lib.ironpulse.utils.LoggedTracer;
import lombok.Getter;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

/** Elevator mechanism using ServoMotorSubsystem with custom functionality for zeroing and characterization. */
public class ElevatorSubsystem extends ServoMotorSubsystem<MotorInputsAutoLogged, MotorIO> {
    private final LinearFilter currentFilter = LinearFilter.movingAverage(ElevatorConfig.ELEVATOR_ZEROING_FILTER_SIZE);
    
    @AutoLogOutput(key = "Elevator/currentFilterValue")
    public double currentFilterValue = 0.0;

    double wantedPosition = 0.0;
    
    @Getter
    @AutoLogOutput(key = "Elevator/zeroing")
    public boolean zeroing = false;
    
    @Getter
    @AutoLogOutput(key = "Elevator/atGoal")
    private boolean atGoal = false;
    
    @Getter
    @AutoLogOutput(key = "Elevator/isGoingUp")
    private boolean isGoingUp = false;
    
    @AutoLogOutput(key = "Elevator/runningCharacterization")
    private boolean runningCharacterization = false;
    
    @AutoLogOutput(key = "Elevator/stopDueToLimit")
    private boolean stopDueToLimit = false;
    
    private double previousWantedPosition = 0.16;

    private final SysIdRoutine m_sysIdRoutine;

    public ElevatorSubsystem() {
        super(
            ElevatorConfig.CONFIG,
            new MotorInputsAutoLogged(),
            createIO(),
            ElevatorParamsNT.asServoMotorParamSources()
        );

        this.m_sysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(
                        Units.Volts.of(ElevatorParamsNT.sysIdRampRateVoltsPerSec.getValue()).per(Units.Second),
                        Units.Volts.of(ElevatorParamsNT.sysIdDynamicVoltage.getValue()),
                        null,
                        (state) -> SignalLogger.writeString("sysid-state", state.toString())
                ),
                new SysIdRoutine.Mechanism(
                        (Voltage volts) -> {
                            io.setInputVoltage(volts.in(Units.Volts));
                            SignalLogger.writeDouble("sysid-elevator-voltage", inputs.appliedVolts, "V");
                            SignalLogger.writeDouble("sysid-elevator-position", getPositionMeters(), "m");
                            SignalLogger.writeDouble("sysid-elevator-velocity", getVelocityMetersPerSec(), "m/s");
                        },
                        null,
                        this
                )
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

        final boolean runningGoal = !stopDueToLimit && !zeroing && !runningCharacterization;
        if (runningGoal) {
            atGoal = positionAtGoal(Rotations.of(ElevatorParamsNT.atGoalToleranceMeters.getValue() / ElevatorConfig.CONFIG.metersPerRotation));
            
            if (wantedPosition != previousWantedPosition) {
                isGoingUp = wantedPosition > previousWantedPosition;
                previousWantedPosition = wantedPosition;
            }
        } else {
            atGoal = false;
        }

        if (runningCharacterization) {
            SignalLogger.writeDouble("elevator-motor-voltage", inputs.appliedVolts, "V");
            SignalLogger.writeDouble("elevator-position", getPositionMeters(), "m");
            SignalLogger.writeDouble("elevator-velocity", getVelocityMetersPerSec(), "m/s");
            SignalLogger.writeDouble("elevator-applied-volts", inputs.appliedVolts, "V");
            SignalLogger.writeDouble("elevator-stator-current", inputs.currentStatorAmps, "A");
        }

        LoggedTracer.record("Elevator");
    }

    public double getElevatorPosition() {
        return getPositionMeters();
    }

    public void setElevatorPosition(DoubleSupplier position) {
        setElevatorPosition(position.getAsDouble());
    }

    public void setElevatorPosition(double meters) {
        double currentPos = getPositionMeters();
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


    public Command zeroElevator() {
        return Commands.startRun(
                        () -> {
                            zeroing = true;
                        },
                        () -> {
                            if (RobotBase.isReal()) {
                                currentFilterValue = currentFilter.calculate(inputs.currentStatorAmps);
                                if (currentFilterValue <= ElevatorParamsNT.zeroingCurrent.getValue()) {
                                    io.setInputVoltage(-1);
                                }
                                if (currentFilterValue > ElevatorParamsNT.zeroingCurrent.getValue()) {
                                    io.setInputVoltage(0);
                                    setCurrentPositionAsZero();
                                    zeroing = false;
                                }
                            } else {
                                setMotionMagicSetpoint(Rotations.of(0));
                                if (Math.abs(getPositionMeters()) < 0.01) {
                                    zeroing = false;
                                }
                            }
                        })
                .until(() -> !zeroing)
                .finallyDo(() -> {
                    zeroing = false;
                });
    }

    public boolean isSafeToFlip() {
        return (getPositionMeters() > ElevatorParamsNT.safeHeightFlip.getValue());
    }
    
    private double getPositionMeters() {
        return inputs.positionRot * ElevatorConfig.CONFIG.metersPerRotation;
    }
    
    private double getVelocityMetersPerSec() {
        return inputs.velocityRotPerSecond * ElevatorConfig.CONFIG.metersPerRotation;
    }
    
    @AutoLogOutput(key = "Elevator/setPoint")
    public double getWantedPosition() {
        return wantedPosition;
    }
}