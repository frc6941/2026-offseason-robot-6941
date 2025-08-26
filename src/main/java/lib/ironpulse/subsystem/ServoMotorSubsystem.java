package lib.ironpulse.subsystem;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import lib.ironpulse.io.MotorIO;
import lib.ironpulse.io.MotorInputsAutoLogged;
import lombok.Getter;

public class ServoMotorSubsystem<T extends MotorInputsAutoLogged, U extends MotorIO> extends MotorSubsystem<T, U> {

    @Getter private Angle positionSetpoint = Degrees.of(0.0);
    private final SubsystemConfig config;
    protected final ParamSources params;
    private final Slot0Configs slot0Configs;
    // Optional linear mechanism mode: meters per motor rotation (> 0 enables linear mode)
    private final double metersPerRotation;

    public ServoMotorSubsystem(SubsystemConfig config, T inputs, U io, ParamSources params) {
      super(config, inputs, io);
      this.config = config;
      this.params = params;
      this.metersPerRotation = config.metersPerRotation;
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
        this.slot0Configs.kG = params.kG();
        io.setNeutralMode(params.isBrake());
        io.updateGains(slot0Configs);
      }
    }

    public boolean positionAtGoal(Angle tolerance){
        return Rotations.of(inputs.positionRot).isNear(positionSetpoint, tolerance);
    }

    // Linear variants (enabled when metersPerRotation > 0)
    public boolean positionAtGoal(Distance tolerance) {
      if (!isLinearMode()) return positionAtGoal(Degrees.of(params.positionAtGoalToleranceDegrees()));
      Distance current = metersFromRotations(Rotations.of(inputs.positionRot));
      Distance goal = metersFromRotations(positionSetpoint);
      return current.isNear(goal, tolerance);
    }

    public boolean positionAtGoal() {
      if (!isLinearMode()) {
        return positionAtGoal(Degrees.of(params.positionAtGoalToleranceDegrees()));
      } else {
        return positionAtGoal(Meters.of(params.positionAtGoalToleranceMeters()));
      }
    }

    public void setCurrentPositionAsZero(){
        io.setCurrentPositionAsZero();
    }

    public void setMotionMagicSetpoint(Angle position){
      if (isLinearMode()) {
        throw new IllegalStateException("Linear mode enabled: use Distance-based setMotionMagicSetpoint");
      }
      io.setMotionMagicSetpoint(position, 
      RotationsPerSecond.of(params.motionMagicVelRPS()),
      RotationsPerSecondPerSecond.of(params.motionMagicAccelRPS2()),
      params.motionMagicJerkRPS3());
    }

    public void setMotionMagicSetpoint(Angle position, AngularVelocity velocity, AngularAcceleration acceleration, double jerk){
      if (isLinearMode()) {
        throw new IllegalStateException("Linear mode enabled: use Distance-based setMotionMagicSetpoint");
      }
      io.setMotionMagicSetpoint(position, velocity, acceleration, jerk);
    }

    // Linear setters
    public void setMotionMagicSetpoint(Distance position) {
      Angle rotations = rotationsFromMeters(position);
      setMotionMagicSetpoint(rotations);
    }
    public void setMotionMagicSetpoint(Distance position, LinearVelocity velocity, LinearAcceleration acceleration, double jerk) {
      Angle rotations = rotationsFromMeters(position);
      // Convert linear velocity/accel to rotational equivalents
      AngularVelocity rotVel = RotationsPerSecond.of(velocity.in(MetersPerSecond) / metersPerRotation);
      AngularAcceleration rotAcc = RotationsPerSecondPerSecond.of(acceleration.in(MetersPerSecondPerSecond) / metersPerRotation);
      setMotionMagicSetpoint(rotations, rotVel, rotAcc, jerk);
    }

    public void setPositionSetpoint(Angle position) {
      if (isLinearMode()) {
        throw new IllegalStateException("Linear mode enabled: use setPositionSetpoint(Distance)");
      }
      io.setPositionSetpoint(position);
    }
    public void setPositionSetpoint(Distance position) {
      io.setPositionSetpoint(rotationsFromMeters(position));
    }

    private boolean isLinearMode() {
      return metersPerRotation > 0.0;
    }
    private Angle rotationsFromMeters(Distance distance) {
      if (!isLinearMode()) {
        throw new IllegalStateException("Linear mode disabled: set metersPerRotation > 0 in SubsystemConfig");
      }
      return Rotations.of(distance.in(Meters) / metersPerRotation);
    }
    private Distance metersFromRotations(Angle rotations) {
      if (!isLinearMode()) {
        throw new IllegalStateException("Linear mode disabled: set metersPerRotation > 0 in SubsystemConfig");
      }
      return Meters.of(rotations.in(Rotations) * metersPerRotation);
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
      default double kG() { return 0.0; }
      default double motionMagicVelRPS() { return 0.0; }
      default double motionMagicAccelRPS2() { return 0.0; }
      default double motionMagicJerkRPS3() { return 0.0; }
      default double positionAtGoalToleranceDegrees() { return 1.0; }
      default double positionAtGoalToleranceMeters() { return 0.005; }
      default boolean isBrake() { return true; }
      /* hook to ParamsNT.isAnyChanged() */
      default boolean hasChanged() { return false; }
    }

}
