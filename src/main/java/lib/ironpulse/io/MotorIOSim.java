package lib.ironpulse.io;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.configs.Slot0Configs;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import lib.ironpulse.subsystem.SubsystemConfig.SimConfig;

public class MotorIOSim implements MotorIO{
    private final DCMotorSim motorSim;
    private final DCMotor motor;
    private final SimConfig cfg;
    private PIDController pidController;
    private SimpleMotorFeedforward driveFF;
    private boolean isDriveCloseLoop = false;
    
    // Motion Magic simulation
    private TrapezoidProfile motionProfile;
    private TrapezoidProfile.State motionProfileGoal;
    private TrapezoidProfile.State motionProfileSetpoint;
    private double lastProfileTime = 0.0;

    public MotorIOSim(SimConfig cfg){
        this.cfg = cfg;
        this.motor = DCMotor.getKrakenX60(1); 
        this.motorSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                motor,
                cfg.MOI.in(KilogramSquareMeters),
                cfg.gearRatio
            ), motor, cfg.stdvs
            );
        pidController = new PIDController(0,0,0);
        driveFF = new SimpleMotorFeedforward(0,0,0);
        pidController.reset();
        
        // Initialize motion profile with default constraints
        motionProfile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(0, 0)
        );
        motionProfileGoal = new TrapezoidProfile.State();
        motionProfileSetpoint = new TrapezoidProfile.State();
    }

    @Override
    public void readInputs(MotorInputs inputs) {
        inputs.velocityRotPerSecond = motorSim.getAngularVelocityRadPerSec();
        inputs.positionRot = motorSim.getAngularPositionRotations();
        inputs.appliedVolts = motorSim.getInputVoltage();
        inputs.motorVolts = motorSim.getInputVoltage();
        inputs.currentSupplyAmps = motorSim.getCurrentDrawAmps();
        inputs.currentStatorAmps = motorSim.getCurrentDrawAmps();
    }

    @Override
    public boolean isConnected() {
        return true;
    }

    @Override
    public void setOpenLoopDutyCycle(double dutyCycle) {
        isDriveCloseLoop = false;
        if (dutyCycle < -1.0 || dutyCycle > 1.0) {
            throw new IllegalArgumentException("Input must be in [-1, 1]");
        }
        motorSim.setInputVoltage(dutyCycle * 12.0);
    }

    @Override
    public void setCurrentPositionAsZero() {
        motorSim.setAngle(0);
    }

    @Override
    public void setCurrentPosition(Angle position) {
        motorSim.setAngle(position.in(Radians));
    }

    @Override
    public void setVelocitySetpoint(AngularVelocity velocity) {
        if (!isDriveCloseLoop) {
            isDriveCloseLoop = true;
            pidController.reset();
        }
        double output = pidController.calculate(motorSim.getAngularVelocityRadPerSec(), velocity.in(RadiansPerSecond));
        double feedforward = driveFF.calculate(velocity.in(RadiansPerSecond));
        motorSim.setInputVoltage(output + feedforward);
    }
    
    @Override
    public void setNeutralMode(boolean wantsBreak) {}

    @Override
    public void setPositionSetpoint(Angle position) {
        if (!isDriveCloseLoop) {
            isDriveCloseLoop = true;
            pidController.reset();
        }
        double output = pidController.calculate(motorSim.getAngularPositionRad(), position.in(Radians));
        double feedforward = driveFF.calculate(0); // Position control uses zero velocity feedforward
        motorSim.setInputVoltage(output + feedforward);
    }

    @Override
    public void setMotionMagicSetpoint(Angle position, double velocity, double acceleration, double jerk) {
        if (!isDriveCloseLoop) {
            isDriveCloseLoop = true;
            pidController.reset();
            lastProfileTime = 0.0;
        }
        
        // Update motion profile constraints with the provided velocity and acceleration
        // Note: Jerk is not directly supported in WPILib's TrapezoidProfile
        motionProfile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(velocity, acceleration)
        );
        
        // Set the goal state
        motionProfileGoal = new TrapezoidProfile.State(position.in(Radians), 0);
        
        // Calculate the current profile setpoint
        // In a real implementation, this would be called periodically (e.g., in periodic())
        // For now, we'll calculate the next setpoint based on a 20ms timestep (50Hz)
        double dt = 0.02;
        lastProfileTime += dt;
        
        TrapezoidProfile.State currentState = new TrapezoidProfile.State(
            motorSim.getAngularPositionRad(),
            motorSim.getAngularVelocityRadPerSec()
        );
        
        motionProfileSetpoint = motionProfile.calculate(dt, currentState, motionProfileGoal);
        
        // Use PID + feedforward to track the profile setpoint
        double positionOutput = pidController.calculate(
            motorSim.getAngularPositionRad(), 
            motionProfileSetpoint.position
        );
        double feedforward = driveFF.calculate(motionProfileSetpoint.velocity);
        
        motorSim.setInputVoltage(positionOutput + feedforward);
    }

    @Override
    public void updateGains(Slot0Configs slot0) {
        pidController.setPID(slot0.kP, slot0.kI, slot0.kD);
        driveFF = new SimpleMotorFeedforward(slot0.kS, slot0.kV, slot0.kA);
    }
}
