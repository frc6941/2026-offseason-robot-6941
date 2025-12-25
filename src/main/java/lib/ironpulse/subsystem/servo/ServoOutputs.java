package lib.ironpulse.subsystem.servo;

import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class ServoOutputs {
    public String currentMode = "VOLTAGE";
    public double setpointDegrees = 0.0;
    public double openLoopValue = 0.0;

    public boolean isZeroing = false;
    public double currentFilterValue = 0.0;
    public boolean isRunningCharacterization = false;
    public boolean positionAtGoal = false;

    public double mechanismPositionMeters = 0.0;
    public double mechanismVelocityMps = 0.0;
}