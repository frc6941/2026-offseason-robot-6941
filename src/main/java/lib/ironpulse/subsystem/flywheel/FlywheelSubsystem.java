package lib.ironpulse.subsystem.flywheel;

import static edu.wpi.first.units.Units.*;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.Slot0Configs;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import lib.ironpulse.io.MotorIO;
import lib.ironpulse.io.MotorInputsAutoLogged;
import lib.ironpulse.subsystem.MotorSubsystem;
import lib.ironpulse.subsystem.SubsystemConfig;
import lombok.Getter;

/**
 * Flywheel subsystem for velocity-controlled mechanisms (shooters, intakes,
 * etc.)
 * Extends MotorSubsystem with velocity control, velocity-at-goal checking, and
 * tunable PID.
 */
public class FlywheelSubsystem<T extends MotorInputsAutoLogged, U extends MotorIO> extends MotorSubsystem<T, U> {

    @Getter
    private FlywheelSetpoint currSetpoint = new FlywheelSetpoint(ModeFlywheel.VELOCITY, RotationsPerSecond.of(0.0),
            () -> 0.0);
    @Getter
    private FlywheelSetpoint prevSetpoint = new FlywheelSetpoint(ModeFlywheel.VELOCITY, RotationsPerSecond.of(0.0),
            () -> 0.0);

    private final SubsystemConfig config;
    protected final FlywheelParamSources params;
    private final Slot0Configs slot0Configs;

    public FlywheelSubsystem(SubsystemConfig config, T inputs, U io, FlywheelParamSources params) {
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

        if (setPointHasChanged()) {
            switch (currSetpoint.modeFlywheel) {
                case VELOCITY:
                    io.setVelocitySetpoint(currSetpoint.velocitySetpt);
                    break;
                case DUTY_CYCLE:
                    runDutyCycle(currSetpoint.openLoop);
                    break;
                case VOLTAGE:
                    runVoltage(currSetpoint.openLoop);
                    break;
                default:
                    io.setVelocitySetpoint(currSetpoint.velocitySetpt);
                    break;
            }
        }
        prevSetpoint = currSetpoint;
    }

    /**
     * Set flywheel velocity setpoint in rotations per second (RPS)
     */
    public void setVelocitySetpoint(AngularVelocity velocity) {
        currSetpoint.modeFlywheel = ModeFlywheel.VELOCITY;
        currSetpoint.velocitySetpt = velocity;
    }

    /**
     * Set flywheel velocity setpoint in rotations per second (RPS)
     */
    public void setVelocitySetpoint(double velocityRPS) {
        setVelocitySetpoint(RotationsPerSecond.of(velocityRPS));
    }

    /**
     * Set flywheel open loop duty cycle [-1.0, 1.0]
     */
    public void setOpenLoopDutyCycle(double dutyCycle) {
        currSetpoint.openLoop = () -> MathUtil.clamp(dutyCycle, -1.0d, 1.0d);
        currSetpoint.modeFlywheel = ModeFlywheel.DUTY_CYCLE;
    }

    /**
     * Set flywheel voltage [-12.0, 12.0]
     */
    public void setVoltage(double voltage) {
        currSetpoint.openLoop = () -> MathUtil.clamp(voltage, -12.0d, 12.0d);
        currSetpoint.modeFlywheel = ModeFlywheel.VOLTAGE;
    }

    /**
     * Check if flywheel velocity is within tolerance of setpoint
     */
    public boolean velocityAtGoal(AngularVelocity tolerance) {
        AngularVelocity current = RotationsPerSecond.of(inputs.velocityRotPerSecond);
        return current.isNear(currSetpoint.velocitySetpt, tolerance);
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

    private enum ModeFlywheel {
        VELOCITY,
        VOLTAGE,
        DUTY_CYCLE
    }

    private boolean setPointHasChanged() {
        return !(currSetpoint.velocitySetpt.equals(prevSetpoint.velocitySetpt) &&
                currSetpoint.modeFlywheel == prevSetpoint.modeFlywheel);
    }

    public static class FlywheelSetpoint {
        private ModeFlywheel modeFlywheel;
        private AngularVelocity velocitySetpt;
        private DoubleSupplier openLoop;

        private FlywheelSetpoint(ModeFlywheel modeFlywheel, AngularVelocity velocitySetpt, DoubleSupplier openLoop) {
            this.modeFlywheel = modeFlywheel;
            this.velocitySetpt = velocitySetpt;
            this.openLoop = openLoop;
        }
    }
}
