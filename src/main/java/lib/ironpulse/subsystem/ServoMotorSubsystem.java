package lib.ironpulse.subsystem;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import lib.ironpulse.io.MotorIO;
import lib.ironpulse.io.MotorInputsAutoLogged;
import lombok.Getter;

public class ServoMotorSubsystem<T extends MotorInputsAutoLogged, U extends MotorIO> extends MotorSubsystem<T, U> {

    @Getter private Angle positionSetpoint = Degrees.of(0.0);
    private final SubsystemConfig config;
    protected final ParamSources params;
    private final Slot0Configs slot0Configs;

    public ServoMotorSubsystem(SubsystemConfig config, T inputs, U io, ParamSources params) {
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
        io.updateGains(slot0Configs);
      }
    }

    public boolean atGoal(Angle tolerance){
        return MotorToMechanismAngle(Rotations.of(inputs.unitPosition)).isNear(positionSetpoint, tolerance);
    }

    public boolean atGoal(){
      return MotorToMechanismAngle(Rotations.of(inputs.unitPosition)).isNear(positionSetpoint, Degrees.of(params.atGoalToleranceDegrees()));
    }

    public void setCurrentPositionAsZero(){
        io.setCurrentPositionAsZero();
    }

    public void setMotionMagicSetpoint(Angle position){
      io.setMotionMagicSetpoint(MechanismToMotorAngle(position), 
      RotationsPerSecond.of(params.motionMagicVelRPS()),
      RotationsPerSecondPerSecond.of(params.motionMagicAccelRPS2()),
      params.motionMagicJerkRPS3());
    }

    public void setMotionMagicSetpoint(Angle position, AngularVelocity velocity, AngularAcceleration acceleration, double jerk){
      io.setMotionMagicSetpoint(MechanismToMotorAngle(position), velocity, acceleration, jerk);
    }

    public void setPositionSetpoint(Angle position) {
      io.setPositionSetpoint(MechanismToMotorAngle(position));
    }

    private Angle MechanismToMotorAngle(Angle position){
      return config.enableRemoteCANcoder?position:position.times(config.SensorToMechanismRatio);
    }

    private Angle MotorToMechanismAngle(Angle position){
      return config.enableRemoteCANcoder?position:position.div(config.SensorToMechanismRatio);
    }

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
      default double atGoalToleranceDegrees() { return 1.0; }
      default boolean isBrake() { return true; }
      default boolean hasChanged() { return false; }
    }

}
