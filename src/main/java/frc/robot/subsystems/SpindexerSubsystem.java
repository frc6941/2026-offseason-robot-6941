package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Configs.SpindexerModeParamsNT;
import frc.robot.subsystems.Configs.SpindexerParamsNT;
import frc.robot.subsystems.ShootingSuperstructure.IdxMode;
import java.util.function.Supplier;
import lib.ironpulse.io.MotorIO;
import lib.ironpulse.io.MotorInputsAutoLogged;
import lib.ironpulse.subsystem.SubsystemConfig;
import lib.ironpulse.subsystem.velocity.VelocityMotorSubsystem;
import lib.ironpulse.subsystem.velocity.VelocityParamSources;
import lib.ironpulse.utils.TimeDelayedBoolean;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class SpindexerSubsystem extends VelocityMotorSubsystem<MotorInputsAutoLogged, MotorIO> {
    private final LinearFilter statorCurrentFilter = LinearFilter.movingAverage(5);
    private final LinearFilter velocityFilter = LinearFilter.movingAverage(5);
    private final TimeDelayedBoolean jamDetectionDelay = new TimeDelayedBoolean(0.0);
    double now = Timer.getTimestamp();

    @Getter
    @AutoLogOutput(key = "Spindexer/state")
    private State currentState = State.OFF;

    @AutoLogOutput(key = "Spindexer/requestedMode")
    private IdxMode requestedMode = IdxMode.OFF;

    @AutoLogOutput(key = "Spindexer/jamCondition")
    private boolean jamCondition = false;

    private double stateStartTimestampSec = Timer.getTimestamp();
    private double offUnjamTimer = Timer.getFPGATimestamp();
    private double jamDetectionLockoutUntilSec = Double.NEGATIVE_INFINITY;
    private double filteredStatorCurrentAmps = 0.0;
    private double filteredVelocityRps = 0.0;

    public SpindexerSubsystem(
            SubsystemConfig config,
            MotorInputsAutoLogged inputs,
            MotorIO io,
            VelocityParamSources params) {
        super(config, inputs, io, params);
    }

    @Override
    public void periodic() {
        now = Timer.getTimestamp();
        super.periodic();

        filteredStatorCurrentAmps = statorCurrentFilter.calculate(getStatorCurrent().magnitude());
        filteredVelocityRps =
                velocityFilter.calculate(Math.abs(getVelocity().in(RotationsPerSecond)));

        Logger.recordOutput(getName() + "/filteredStatorCurrentAmps", filteredStatorCurrentAmps);
        Logger.recordOutput(getName() + "/filteredVelocityRps", filteredVelocityRps);
        Logger.recordOutput(
                getName() + "/jamDetectionLockedOut",
                Timer.getTimestamp() < jamDetectionLockoutUntilSec);
        SmartDashboard.putBoolean("Spindexer/jamCondition", jamCondition);

        jamCondition = isJamConditionMet(now);

        handleStateTransitions();
    }

    private void handleStateTransitions() {
        switch (currentState) {
            case FEED:
                if (jamCondition
                        && jamDetectionDelay.update(
                                jamCondition, SpindexerParamsNT.unjammTriggerSec.getValue())) {
                    transitionTo(State.UNJAM_REVERSE, now);
                }
                break;

            case UNJAM_REVERSE:
                if (now - stateStartTimestampSec >= SpindexerParamsNT.unjammTimeoutSec.getValue()) {
                    transitionTo(State.FEED, now);
                    armJamDetectionLockout(now);
                }
                break;

            default:
                break;
        }
    }

    public Command runState(Supplier<IdxMode> requestedModeSupplier) {
        return runVelVolt(
                () -> {
                    requestedMode = requestedModeSupplier.get();
                    return updateStateAndGetVelocity(requestedMode);
                });
    }

    private AngularVelocity updateStateAndGetVelocity(IdxMode wantedMode) {
        double now = Timer.getTimestamp();

        switch (wantedMode) {
            case OFF:
                jamCondition = false;
                transitionToIfNeeded(State.OFF, now);
                break;
            case FORCE_FEED:
                jamCondition = false;
                transitionToIfNeeded(State.MANUAL_FEED, now);
                break;
            case REVERSE:
                jamCondition = false;
                transitionToIfNeeded(State.MANUAL_REVERSE, now);
                break;
            case FEED:
                switch (currentState) {
                    case OFF:
                    case MANUAL_FEED:
                    case MANUAL_REVERSE:
                        jamCondition = false;
                        transitionToIfNeeded(State.FEED, now);
                        armJamDetectionLockout(now);
                        break;

                    case FEED:
                        break;

                    case UNJAM_REVERSE:
                        break;
                }
                break;
        }

        return switch (currentState) {
            case OFF -> SpindexerParamsNT.periodicUnjamEnabled.getValue()
                    ? (RotationsPerSecond.of(
                            (0.5
                                    * Math.sin(
                                            2
                                                    * Math.PI
                                                    / SpindexerParamsNT.periodicUnjamIntervalSec
                                                            .getValue()
                                                    * now))))
                    : RotationsPerSecond.of(SpindexerModeParamsNT.idleRPS.getValue());
            case FEED, MANUAL_FEED -> RotationsPerSecond.of(
                    SpindexerModeParamsNT.feedRPS.getValue());
            case UNJAM_REVERSE, MANUAL_REVERSE -> RotationsPerSecond.of(
                    SpindexerModeParamsNT.revRPS.getValue());
        };
    }

    private boolean isJamConditionMet(double now) {
        if (now < jamDetectionLockoutUntilSec) {
            return false;
        }

        return filteredStatorCurrentAmps > SpindexerParamsNT.unjammTriggerAmps.getValue()
                && filteredVelocityRps < SpindexerParamsNT.unjammTriggerBelowRps.getValue();
    }

    private void armJamDetectionLockout(double now) {
        jamDetectionLockoutUntilSec = now + SpindexerParamsNT.unjammLockoutSec.getValue();
        jamDetectionDelay.reset();
    }

    private void transitionToIfNeeded(State nextState, double now) {
        if (currentState != nextState) {
            transitionTo(nextState, now);
        }
    }

    private void transitionTo(State nextState, double now) {
        currentState = nextState;
        stateStartTimestampSec = now;
        jamDetectionDelay.reset();
    }

    public enum State {
        OFF,
        FEED,
        MANUAL_FEED,
        UNJAM_REVERSE,
        MANUAL_REVERSE
    }
}
