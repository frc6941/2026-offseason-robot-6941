package lib.ironpulse.io;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.Slot0Configs;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import lib.ironpulse.subsystem.SubsystemConfig;
import lib.ironpulse.subsystem.SubsystemConfig.SimConfig;

import java.util.Random;

//TODO:NOT Working
public class MotorIOSim implements MotorIO {

    private DCMotorSim dcMotorSim;
    private DCMotor dcMotor;
    private SimConfig cfg;
    private final SubsystemConfig subsystemConfig;
    private SimpleMotorFeedforward feedforward;
    private PIDController pidController;
    private ProfiledPIDController profiledPidController;
    private final Random random = new Random();

    private double appliedVolts;
    private boolean isCloseLoop = false;
    private double kg = 0.0;

    // Motion Magic support
    private TrapezoidProfile motionProfile;
    private TrapezoidProfile.State motionProfileGoal;
    private double motionProfileStartTime = -1;
    private TrapezoidProfile.State lastSetpoint;

    public MotorIOSim(SubsystemConfig cfg) {
        this.cfg = cfg.simConfig;
        this.subsystemConfig = cfg;
        this.dcMotor = DCMotor.getKrakenX60Foc(1);
        this.dcMotorSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(dcMotor, this.cfg.MOI.magnitude(), this.cfg.gearRatio), dcMotor,
                this.cfg.stdvs);
        initializeControllers();
    }

    private void initializeControllers() {
        pidController = new PIDController(0, 0, 0);
        feedforward = new SimpleMotorFeedforward(0, 0, 0);

        // ProfiledPID for position control with trajectory generation
        // Use constraints from SimConfig, or default if not set
        TrapezoidProfile.Constraints constraints = (cfg.profile != null)
                ? cfg.profile
                : new TrapezoidProfile.Constraints(50.0, 100.0);
        profiledPidController = new ProfiledPIDController(0, 0, 0, constraints);
    }

    @Override
    public void readInputs(MotorInputs inputs) {
        dcMotorSim.setInputVoltage(appliedVolts);
        dcMotorSim.update(0.02);
        inputs.appliedVolts = appliedVolts;

        double noiseMultiplier = 0.95 + Math.sqrt(0.1) * random.nextGaussian();
        inputs.currentStatorAmps = dcMotorSim.getCurrentDrawAmps() * noiseMultiplier;
        inputs.currentSupplyAmps = dcMotorSim.getCurrentDrawAmps();
        inputs.positionRot = dcMotorSim.getAngularPositionRotations();
        inputs.velocityRotPerSecond = dcMotorSim.getAngularVelocityRadPerSec() / (2 * Math.PI);
        inputs.motorVolts = dcMotor.getVoltage(dcMotorSim.getTorqueNewtonMeters(),
                dcMotorSim.getAngularVelocityRadPerSec()) * cfg.gearRatio;
    }

    @Override
    public void setOpenLoopDutyCycle(double dutyCycle) {
        isCloseLoop = false;
        appliedVolts = MathUtil.clamp(dutyCycle * 12, -12.0f, 12.0f);
    }

    @Override
    public void setInputVoltage(double voltage) {
        isCloseLoop = false;
        appliedVolts = MathUtil.clamp(voltage, -12.0f, 12.0f);
    }

    @Override
    public void setVelocitySetpoint(AngularVelocity velocity) {
        if (!isCloseLoop) {
            isCloseLoop = true;
            pidController.reset();
        }
        double fb = pidController.calculate(dcMotorSim.getAngularVelocityRadPerSec() / (2 * Math.PI),
                velocity.in(RotationsPerSecond));
        double ff = feedforward.calculate(velocity.in(RotationsPerSecond));
        appliedVolts = MathUtil.clamp(fb + ff, -12.0, 12.0);
    }

    @Override
    public void setPositionSetpoint(Angle position) {
        if (!isCloseLoop) {
            isCloseLoop = true;
            profiledPidController.reset(dcMotorSim.getAngularPositionRotations());
        }

        double currentPosition = dcMotorSim.getAngularPositionRotations();
        double targetPosition = position.in(Rotations);

        // ProfiledPID calculates feedback with automatic trajectory generation
        double fb = profiledPidController.calculate(currentPosition, targetPosition);

        // Feedforward based on the profiled setpoint velocity + gravity compensation
        TrapezoidProfile.State setpoint = profiledPidController.getSetpoint();
        double ff = feedforward.calculate(setpoint.velocity) + kg;

        // Apply voltage
        appliedVolts = MathUtil.clamp(fb + ff, -12.0, 12.0);
    }

    @Override
    public void setMotionMagicSetpoint(Angle position, double velocity, double acceleration, double jerk) {
        if (!isCloseLoop) {
            isCloseLoop = true;
            pidController.reset();
        }

        double currentTime = Timer.getFPGATimestamp();
        double currentPosition = dcMotorSim.getAngularPositionRotations();
        double currentVelocity = dcMotorSim.getAngularVelocityRadPerSec() / (2 * Math.PI);

        TrapezoidProfile.State currentState = new TrapezoidProfile.State(currentPosition, currentVelocity);

        boolean needsNewProfile = motionProfile == null ||
                !(motionProfileGoal.position - (position.in(Rotations)) == 0.0003) ||
                motionProfileStartTime < 0;

        if (needsNewProfile) {
            TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(velocity, acceleration);
            motionProfile = new TrapezoidProfile(constraints);
            motionProfileGoal = new TrapezoidProfile.State(position.in(Rotations), 0.0);
            motionProfileStartTime = currentTime;
            lastSetpoint = currentState;
        }

        double elapsedTime = currentTime - motionProfileStartTime;
        TrapezoidProfile.State setpoint = motionProfile.calculate(elapsedTime, lastSetpoint, motionProfileGoal);

        double fb = pidController.calculate(currentPosition, setpoint.position);

        double ff = feedforward.calculate(setpoint.velocity);

        appliedVolts = MathUtil.clamp(fb + ff + kg, -12.0, 12.0);

        lastSetpoint = setpoint;
    }

    @Override
    public void setCurrentPositionAsZero() {
        setCurrentPosition(Radians.of(0));
    }

    @Override
    public void setCurrentPosition(Angle position) {
        double positionRad = position.in(Radians);
        double currentVelocityRadPerSec = dcMotorSim.getAngularVelocityRadPerSec();

        dcMotorSim.setState(positionRad, currentVelocityRadPerSec);
    }

    @Override
    public void updateGains(Slot0Configs slot0) {
        double kp = slot0.kP;
        double ki = slot0.kI;
        double kd = slot0.kD;
        double ka = slot0.kA;
        double kv = slot0.kV;
        double ks = slot0.kS;
        kg = slot0.kG;

        // Update velocity PID controller
        pidController.setPID(kp, ki, kd);
        feedforward.setKa(ka);
        feedforward.setKs(ks);
        feedforward.setKv(kv);

        // Update position ProfiledPID controller
        profiledPidController.setPID(kp, ki, kd);

        // kg is stored separately and added in position/motion magic control
    }

}
