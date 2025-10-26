package lib.ironpulse.subsystem;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.units.measure.AngularVelocity;
import lib.ironpulse.io.MotorIO;
import lib.ironpulse.io.MotorInputsAutoLogged;
import lombok.Getter;

/**
 * Flywheel subsystem for velocity-controlled mechanisms (shooters, intakes, etc.)
 * Extends MotorSubsystem with velocity control, velocity-at-goal checking, and tunable PID.
 */
public class FlywheelSubsystem<T extends MotorInputsAutoLogged, U extends MotorIO> extends MotorSubsystem<T, U> {

    @Getter private AngularVelocity velocitySetpoint = RotationsPerSecond.of(0.0);
    private final SubsystemConfig config;
    protected final ParamSources params;
    private final Slot0Configs slot0Configs;

    public FlywheelSubsystem(SubsystemConfig config, T inputs, U io, ParamSources params) {
        super(config, inputs, io);
        this.config = config;
        this.params = params;
        slot0Configs = new Slot0Configs();
        slot0Configs.kP = params.kP();
        slot0Configs.kI = params.kI();
        slot0Configs.kD = params.kD();
        slot0Configs.kA = params.kA();
        slot0Configs.kV = params.kV();
        slot0Configs.kS = params.kS();
        io.updateGains(slot0Configs);
    }

    @Override
    public void periodic() {
        super.periodic();
        if (params.hasChanged()) {
            this.slot0Configs.kP = params.kP();
            this.slot0Configs.kI = params.kI();
            this.slot0Configs.kD = params.kD();
            this.slot0Configs.kA = params.kA();
            this.slot0Configs.kV = params.kV();
            this.slot0Configs.kS = params.kS();
            io.setNeutralMode(params.isBrake());
            io.updateGains(slot0Configs);
        }
    }

    /**
     * Set flywheel velocity setpoint in rotations per second (RPS)
     */
    public void setVelocitySetpoint(AngularVelocity velocity) {
        this.velocitySetpoint = velocity;
        io.setVelocitySetpoint(velocity);
    }

    /**
     * Set flywheel velocity setpoint in rotations per second (RPS)
     */
    public void setVelocitySetpoint(double velocityRPS) {
        setVelocitySetpoint(RotationsPerSecond.of(velocityRPS));
    }

    /**
     * Check if flywheel velocity is within tolerance of setpoint
     */
    public boolean velocityAtGoal(AngularVelocity tolerance) {
        AngularVelocity current = RotationsPerSecond.of(inputs.velocityRotPerSecond);
        return current.isNear(velocitySetpoint, tolerance);
    }

    /**
     * Check if flywheel velocity is within default tolerance of setpoint
     */
    public boolean velocityAtGoal() {
        return velocityAtGoal(RotationsPerSecond.of(params.velocityAtGoalToleranceRPS()));
    }

    /**
     * Get current velocity in rotations per second
     */
    public AngularVelocity getVelocity() {
        return RotationsPerSecond.of(inputs.velocityRotPerSecond);
    }

    /**
     * Check if motor is connected
     */
    public boolean isConnected() {
        return io.isConnected();
    }

    /**
     * Stop the flywheel
     */
    public void stop() {
        setVelocitySetpoint(0.0);
    }

    //hack to hook NTParameterProcessor to generate ParamSources for uses in subsystems
    //REMEMBER to update NTParameterProcessor when adding new fields to ParamSources
    public interface ParamSources {
        double kP();
        double kI();
        double kD();
        default double kA() { return 0.0; }
        default double kV() { return 0.0; }
        default double kS() { return 0.0; }
        default double velocityAtGoalToleranceRPS() { return 1.0; }
        default boolean isBrake() { return false; } // Typically coast for flywheels
        /* hook to ParamsNT.isAnyChanged() */
        default boolean hasChanged() { return false; }
    }
}
