package lib.ironpulse.io;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;

/**
 * Follow Team 254 style for ownership (subsystem logs)
 */
public interface MotorIO {
  // Read
  void readInputs(MotorInputs inputs);

  // Write (mechanism units)
  void setOpenLoopDutyCycle(double dutyCycle);
  void setPositionSetpoint(Angle position);
  void setMotionMagicSetpoint(Angle position, double velocity, double acceleration, double jerk);
  void setNeutralMode(boolean wantsBreak);
  void setVelocitySetpoint(AngularVelocity velocity);
  void setCurrentPositionAsZero();
  void setCurrentPosition(Angle position);
  void setEnableSoftLimits(boolean forward, boolean reverse);
  void updateGains(Slot0Configs slot0);
}