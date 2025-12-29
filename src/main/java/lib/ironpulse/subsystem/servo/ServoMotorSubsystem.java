package lib.ironpulse.subsystem.servo;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import lib.ironpulse.io.MotorIO;
import lib.ironpulse.io.MotorInputsAutoLogged;
import lib.ironpulse.subsystem.MotorSubsystem;
import lib.ironpulse.subsystem.SubsystemConfig;
import lib.ironpulse.utils.LoggedTracer;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.*;

public class ServoMotorSubsystem<T extends MotorInputsAutoLogged, U extends MotorIO, M extends Measure<?>> extends MotorSubsystem<T, U> {

    protected final ServoParamSources params;
    private final Angle zeroOffset;
    private final Slot0Configs slot0Configs;
    private final MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
    private final M mechanismUnitPerRotation;
    private M currSetpoint;
    private LinearFilter currentFilter = LinearFilter.movingAverage(5);
    private double currentFilterValue = 0.0;

    /**
     * Creates a new generic ServoMotorSubsystem.
     *
     * @param config                   The subsystem configuration (IDs, buses, ratios).
     * @param inputs                   The autologged inputs for the mechanism.
     * @param io                       The IO interface for the hardware.
     * @param params                   The parameter sources (PID, MotionMagic limits).
     * @param initialSetpoint          The starting position in mechanism units (M).
     * @param mechanismUnitPerRotation The amount of mechanism movement per ONE
     *                                 motor rotation. Example (Elevator):
     *                                 Meters.of(ElevatorConfig.METERS_PER_ROTATION) Example (Pivot):
     *                                 Degrees.of(360)
     */
    public ServoMotorSubsystem(SubsystemConfig config, T inputs, U io, ServoParamSources params, M initialSetpoint, M mechanismUnitPerRotation) {
        super(config, inputs, io);
        this.params = params;
        this.zeroOffset = config.zeroOffset;
        this.mechanismUnitPerRotation = mechanismUnitPerRotation;
        this.currSetpoint = initialSetpoint;
        slot0Configs = new Slot0Configs();
        slot0Configs.kP = params.kP();
        slot0Configs.kI = params.kI();
        slot0Configs.kD = params.kD();
        slot0Configs.kA = params.kA();
        slot0Configs.kV = params.kV();
        slot0Configs.kS = params.kS();
        slot0Configs.kG = params.kG();
        io.updateGains(slot0Configs);
    }

    @SuppressWarnings("unchecked")
    private Angle toAngle(M mechanismValue) {
        Unit mechanismUnit = (Unit) mechanismUnitPerRotation.unit();
        return Rotations.of(((Measure) mechanismValue).in(mechanismUnit) / ((Measure) mechanismUnitPerRotation).in(mechanismUnit));
    }

    @SuppressWarnings("unchecked")
    private M fromAngle(Angle motorAngle) {
        return (M) mechanismUnitPerRotation.times(motorAngle.in(Rotations));
    }

    @Override
    @SuppressWarnings("unchecked")
    public void periodic() {

        super.periodic();
        if (params.hasChanged()) {
            this.slot0Configs.kP = params.kP();
            this.slot0Configs.kI = params.kI();
            this.slot0Configs.kD = params.kD();
            this.slot0Configs.kA = params.kA();
            this.slot0Configs.kV = params.kV();
            this.slot0Configs.kS = params.kS();
            this.slot0Configs.kG = params.kG();
            io.setNeutralMode(params.isBrake());
            io.updateGains(slot0Configs);
        }

        Logger.recordOutput(config.name + "/atGoal", positionAtGoal());
        Logger.recordOutput(config.name + "/currPosition", ((Measure) getCurrPos()).in((Unit) mechanismUnitPerRotation.unit()));

        LoggedTracer.record(config.name);
    }

    @SuppressWarnings("unchecked")
    public boolean positionAtGoal(M tolerance) {
        // Cast to raw Measure to bypass generic issues with isNear in some WPILib versions
        return ((Measure) getCurrPos()).isNear((Measure) currSetpoint, (Measure) tolerance);
    }

    @SuppressWarnings("unchecked")
    public boolean positionAtGoal() {
        try {
            return positionAtGoal((M) Degrees.of(params.positionAtGoalToleranceDegrees()));
        } catch (ClassCastException e) {
            return positionAtGoal((M) Meters.of(params.positionAtGoalToleranceMeters()));
        }
    }

    public Command runMotionMagic(M setPoint) {
        return Commands.run(
                () -> {
                    motionMagicConfigs.MotionMagicAcceleration = params.motionMagicAccelRPS2();
                    motionMagicConfigs.MotionMagicCruiseVelocity = params.motionMagicVelRPS();
                    motionMagicConfigs.MotionMagicJerk = params.motionMagicJerkRPS3();
                    Unit mechanismUnit = (Unit) mechanismUnitPerRotation.unit();
                    Logger.recordOutput(config.name + "/mode", "MOTIONMAGIC");
                    Logger.recordOutput(config.name + "/setPoint", ((Measure) setPoint).in(mechanismUnit));
                    io.setMotionMagicSetpoint(
                            toAngle(setPoint).minus(zeroOffset),
                            motionMagicConfigs.MotionMagicCruiseVelocity,
                            motionMagicConfigs.MotionMagicAcceleration,
                            motionMagicConfigs.MotionMagicJerk);
                    currSetpoint = setPoint;
                },
                this);
    }

    public Command runMotionMagic(M setPoint, double velocity, double acceleration, double jerk) {
        return Commands.run(
                () -> {
                    motionMagicConfigs.MotionMagicAcceleration = acceleration;
                    motionMagicConfigs.MotionMagicCruiseVelocity = velocity;
                    motionMagicConfigs.MotionMagicJerk = jerk;
                    Unit mechanismUnit = (Unit) mechanismUnitPerRotation.unit();
                    Logger.recordOutput(config.name + "/mode", "MOTIONMAGIC");
                    Logger.recordOutput(config.name + "/setPoint", ((Measure) setPoint).in(mechanismUnit));
                    io.setMotionMagicSetpoint(
                            toAngle(setPoint).minus(zeroOffset),
                            motionMagicConfigs.MotionMagicCruiseVelocity,
                            motionMagicConfigs.MotionMagicAcceleration,
                            motionMagicConfigs.MotionMagicJerk);
                    currSetpoint = setPoint;
                },
                this);
    }

    public Command runPosition(M setPoint) {
        return Commands.run(
                () -> {
                    Unit mechanismUnit = (Unit) mechanismUnitPerRotation.unit();
                    Logger.recordOutput(config.name + "/mode", "POSITION");
                    Logger.recordOutput(config.name + "/setPoint", ((Measure) setPoint).in(mechanismUnit));
                    io.setPositionSetpoint(toAngle(setPoint).minus(zeroOffset));
                    currSetpoint = setPoint;
                },
                this);
    }

    @Override
    public Command runDutyCycle(DoubleSupplier dutyCycle) {
        return Commands.run(
                () -> {
                    double out = MathUtil.clamp(dutyCycle.getAsDouble(), -1.0d, 1.0d);
                    Logger.recordOutput(config.name + "/mode", "DUTY_CYCLE");
                    Logger.recordOutput(config.name + "/setPoint", out);
                    io.setOpenLoopDutyCycle(out);
                },
                this);
    }

    public Command runDutyCycle(double dutyCycle) {
        return runDutyCycle(() -> dutyCycle);
    }

    @Override
    public Command runVoltage(DoubleSupplier voltage) {
        return Commands.run(
                () -> {
                    double out = MathUtil.clamp(voltage.getAsDouble(), -12.0d, 12.0d);
                    Logger.recordOutput(config.name + "/mode", "VOLTAGE");
                    Logger.recordOutput(config.name + "/setPoint", out);
                    io.setVoltage(out);
                },
                this);
    }

    public Command runVoltage(double voltage) {
        return runVoltage(() -> voltage);
    }

    /**
     * Returns a command that zeroes the mechanism by driving it until a current
     * spike is detected.
     *
     * @return The zeroing command.
     */
    public Command zeroCommand() {
        double zeroVoltage = MathUtil.clamp(config.zeroingConfig.zeroingVoltage, -12.0d, 12.0d);

        Command init = Commands.runOnce(
                () -> {
                    currentFilter = LinearFilter.movingAverage(config.zeroingConfig.zeroingFilterSize);
                    currentFilterValue = 0.0;
                },
                this);

        Runnable stop = () -> {
            Logger.recordOutput(config.name + "/mode", "VOLTAGE");
            Logger.recordOutput(config.name + "/setPoint", 0.0);
            io.setVoltage(0.0);
        };

        Command realZero = init
                .andThen(
                        Commands.deadline(
                                Commands.run(() -> currentFilterValue = currentFilter.calculate(inputs.currentStatorAmps))
                                        .until(() -> currentFilterValue > config.zeroingConfig.zeroingCurrentLimit),
                                runVoltage(zeroVoltage)))
                .andThen(Commands.runOnce(() -> io.setCurrentPositionAsZero(), this))
                .finallyDo(stop);

        Command simZero = init
                .andThen(runMotionMagic(fromAngle(Rotations.of(0))))
                .until(() -> Math.abs(inputs.positionRot) < 0.01)
                .finallyDo(stop);

        return RobotBase.isReal() ? realZero : simZero;
    }

    public M getCurrPos() {
        return fromAngle(Rotations.of(inputs.positionRot + zeroOffset.in(Rotations)));
    }

    public M getCurrSetpoint() {
        return currSetpoint;
    }

}
