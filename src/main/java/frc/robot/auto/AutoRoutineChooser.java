package frc.robot.auto;

import static frc.robot.auto.AutoActions.*;
import static frc.robot.auto.AutoRoutines.*;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import lombok.Getter;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class AutoRoutineChooser {

    @Getter
    private static final LoggedDashboardChooser<AutoType> autoChooser =
            new LoggedDashboardChooser<>("Auto Type Chooser222");

    @Getter
    private static final LoggedDashboardChooser<AutoRoutine> routineChooser =
            new LoggedDashboardChooser<>("Auto Routine Chooser222");

    private static final Alert invalidConfigAlert =
            new Alert("Auto configuration is invalid", Alert.AlertType.kError);

    public static void init() {
        // Initialize main auto type chooser
        for (AutoType type : AutoType.values()) {
            autoChooser.addOption(type.toString(), type);
        }

        // Initialize routine chooser with all possible routines
        for (AutoRoutine routine : AutoRoutine.values()) {
            routineChooser.addOption(routine.toString(), routine);
        }
    }

    public static Command buildAuto() {
        return switch (autoChooser.get()) {
            case TEST -> buildTestRoutine();
            case COMPETITION -> buildCompetitionRoutine();
            default -> Commands.none();
        };
    }

    private static Command buildTestRoutine() {
        return testRoutine();
    }

    private static Command buildCompetitionRoutine() {
        AutoRoutine selected = routineChooser.get();

        if (selected == null) {
            invalidConfigAlert.set(true);
            return Commands.none();
        }

        invalidConfigAlert.set(false);

        return switch (selected) {
                // LEFT side routines
            case LEFT_FAST_CLIMB -> leftFastClimb();
            case LEFT_FAST_FUEL -> leftFastFuel();
            case LEFT_NORMAL_CLIMB -> leftNormalClimb();
            case LEFT_NORMAL_FUEL -> leftNormalFuel();
            case LEFT_LONG_FUEL -> leftLongFuel();

                // RIGHT side routines
            case RIGHT_FAST_CLIMB -> rightFastClimb();
            case RIGHT_FAST_FUEL -> rightFastFuel();
            case RIGHT_NORMAL_CLIMB -> rightNormalClimb();
            case RIGHT_NORMAL_FUEL -> rightNormalFuel();
            case RIGHT_LONG_FUEL -> rightLongFuel();

            default -> Commands.none();
        };
    }

    public enum AutoType {
        TEST,
        COMPETITION
    }

    public enum AutoRoutine {
        // Test
        TEST_ROUTINE,

        // LEFT side
        LEFT_FAST_CLIMB,
        LEFT_FAST_FUEL,
        LEFT_NORMAL_CLIMB,
        LEFT_NORMAL_FUEL,
        LEFT_LONG_FUEL,

        // RIGHT side
        RIGHT_FAST_CLIMB,
        RIGHT_FAST_FUEL,
        RIGHT_NORMAL_CLIMB,
        RIGHT_NORMAL_FUEL,
        RIGHT_LONG_FUEL
    }
}
