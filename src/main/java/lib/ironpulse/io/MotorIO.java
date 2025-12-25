package lib.ironpulse.io;

import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

/**
 * Follow Team 254 style for ownership (subsystem logs)
 */
public interface MotorIO {
  // Read
  default void readInputs(MotorInputs inputs) {};

  default void readFollowerInputs(MotorInputs[] inputs) {};

  // Write (mechanism units)
  default void setOpenLoopDutyCycle(double dutyCycle) {};

  default void setInputVoltage(double voltage) {};

  default void setPositionSetpoint(Angle position) {};

  default void setMotionMagicSetpoint(Angle position, double velocity, double acceleration, double jerk) {};

  default void setNeutralMode(boolean wantsBreak) {};

  default void setVelocitySetpoint(AngularVelocity velocity) {};

  default void setCurrentPositionAsZero() {};

  default void setVoltage(double voltage) {};

  default void setCurrentPosition(Angle position) {};

  default void setEnableSoftLimits(boolean forward, boolean reverse) {};

  default void updateGains(Slot0Configs slot0) {};

  default boolean isConnected() {return true;};
}