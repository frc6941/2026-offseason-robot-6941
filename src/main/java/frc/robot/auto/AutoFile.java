package frc.robot.auto;

import static frc.robot.auto.AutoActions.*;
import static frc.robot.auto.AutoRoutines.*;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import java.io.File;
import java.io.IOException;
import java.util.Collections;
import java.util.HashMap;
import java.util.Map;
import lombok.Getter;
import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class AutoFile {
    private static final Map<String, PathPlannerPath> autoPaths = new HashMap<>();

    @Getter
    private static final LoggedDashboardChooser<AutoType> autoChooser =
            new LoggedDashboardChooser<AutoType>("Auto Type Chooser");

    private static final LoggedDashboardChooser<AutoSide> sideChooser =
            new LoggedDashboardChooser<AutoSide>("Auto Side Chooser");
    private static final LoggedDashboardChooser<SweepMode> sweepModeChooser =
            new LoggedDashboardChooser<SweepMode>("Sweep Mode Chooser");
    private static final LoggedDashboardChooser<EndBehaviour> endBehaviourChooser =
            new LoggedDashboardChooser<EndBehaviour>("End Behaviour Chooser");

    private static final Alert competitionNotSelectedAlert =
            new Alert("Competition auto is not running", Alert.AlertType.kWarning);
    private static final Alert invalidConfigAlert =
            new Alert(
                    "Auto configuration is invalid, fallback to Command.None()",
                    Alert.AlertType.kError);

    private static <E extends Enum<E>> void initializeChooser(
            LoggedDashboardChooser<E> chooser, E[] values) {
        chooser.onChange((value) -> verifyOption());
        for (E e : values) {
            chooser.addOption(e.toString(), e);
        }
    }

    public static void init() {
        initializeAutoPaths();
        initializeChooser(autoChooser, AutoType.values());
        initializeChooser(sideChooser, AutoSide.values());
        initializeChooser(endBehaviourChooser, EndBehaviour.values());
        initializeChooser(sweepModeChooser, SweepMode.values());
    }

    private static void initializeAutoPaths() {
        File[] files = new File(Filesystem.getDeployDirectory(), "pathplanner/paths").listFiles();
        assert files != null;
        for (File file : files) {
            try {
                // path files without extension
                PathPlannerPath path =
                        PathPlannerPath.fromPathFile(file.getName().replaceFirst("[.][^.]+$", ""));
                autoPaths.put(path.name, path);
            } catch (IOException | ParseException e) {
                throw new IllegalArgumentException(
                        "Failed to parse path file: " + file.getName(), e);
            }
        }
    }

    private static PathPlannerPath getAutoPath(String path) {
        assert autoPaths.containsKey(path);
        return autoPaths.get(path);
    }

    public static Command buildTest() {
        return AutoActions.drivePastSlope(false, true);
    }

    public static Command buildAuto() {
        return switch (autoChooser.get()) {
            case TEST -> buildTest();
            case COMPETITION -> buildCompetition();
        };
    }

    private static void verifyOption() {
        if (autoChooser.get() == AutoType.COMPETITION) {
            competitionNotSelectedAlert.set(false);
            invalidConfigAlert.set(
                    sweepModeChooser.get() == SweepMode.LONG
                            && endBehaviourChooser.get() != EndBehaviour.FUEL);
        } else {
            competitionNotSelectedAlert.set(true);
            invalidConfigAlert.set(false);
        }
    }

    private static Command buildCompetition() {
        if (invalidConfigAlert.get()) return Commands.none();
        boolean isLeft = sideChooser.get() == AutoSide.LEFT;
        String sweepPathName =
                switch (sweepModeChooser.get()) {
                    case FAST -> "quickSweepRight";
                    case LONG -> "longSweepRight";
                    case NORMAL -> "sweepRight";
                };
        return Commands.defer(
                () ->
                        Commands.sequence(
                                Commands.sequence(
                                        // sweep
                                        drivePastSlope(isLeft, true),
                                        Commands.deadline(
                                                followPathFile(sweepPathName, isLeft), intake()),
                                        drivePastSlope(isLeft, false),
                                        shoot()),

                                // station
                                Commands.sequence(
                                                new ConditionalCommand(
                                                        Commands.deadline(
                                                                allignToDepot(),
                                                                oscillateIntakeFeed()),
                                                        Commands.deadline(
                                                                allignToStation(),
                                                                oscillateIntakeFeed()),
                                                        () -> isLeft),
                                                shoot())
                                        .onlyIf(
                                                () ->
                                                        endBehaviourChooser.get()
                                                                != EndBehaviour.CLIMB),

                                // climb
                                Commands.parallel(
                                                oscillateIntakeFeed(),
                                                shoot().withTimeout(2)
                                                        .onlyIf(
                                                                () ->
                                                                        endBehaviourChooser.get()
                                                                                == EndBehaviour
                                                                                        .FUEL_CLIMB),
                                                climbUp(),
                                                alignToClimb(isLeft))
                                        .onlyIf(
                                                () ->
                                                        endBehaviourChooser.get()
                                                                != EndBehaviour.FUEL)),
                Collections.singleton(swerve));
    }

    private enum AutoType {
        COMPETITION,
        TEST
    }

    private enum AutoSide {
        RIGHT,
        LEFT
    }

    private enum SweepMode {
        FAST,
        LONG,
        NORMAL
    }

    private enum EndBehaviour {
        CLIMB,
        FUEL,
        FUEL_CLIMB
    }
}
