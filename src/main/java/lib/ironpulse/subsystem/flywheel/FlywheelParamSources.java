package lib.ironpulse.subsystem.flywheel;

// hack to hook NTParameterProcessor to generate ParamSources for uses in subsystems
// REMEMBER to update NTParameterProcessor when adding new fields to ParamSources
public interface FlywheelParamSources {
    double kP();

    double kI();

    double kD();

    default double kA() {
        return 0.0;
    }

    default double kV() {
        return 0.0;
    }

    default double kS() {
        return 0.0;
    }

    default double velocityAtGoalToleranceRPS() {
        return 1.0;
    }

    default boolean isBrake() {
        return false;
    } // Typically coast for flywheels

    /* hook to ParamsNT.isAnyChanged() */
    default boolean hasChanged() {
        return false;
    }
}
