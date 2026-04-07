package frc.robot.auto;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.auto.AutoActions.*;
import static frc.robot.auto.AutoRoutines.*;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.ShootingSubsystem.ShootingSuperstructure.IdxMode;
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
    private static final Timer autoTimer = new Timer();

    @Getter
    private static final LoggedDashboardChooser<AutoType> autoChooser =
            new LoggedDashboardChooser<AutoType>("Auto Type Chooser");

    private static final LoggedDashboardChooser<AutoSide> sideChooser =
            new LoggedDashboardChooser<AutoSide>("Auto Side Chooser");
    private static final LoggedDashboardChooser<SweepMode> sweepModeChooser =
            new LoggedDashboardChooser<SweepMode>("Sweep Mode Chooser");
    private static final LoggedDashboardChooser<EndBehaviour> endBehaviourChooser =
            new LoggedDashboardChooser<EndBehaviour>("End Behaviour Chooser");
    private static final LoggedDashboardChooser<Integer> sweepCyclesChooser =
            new LoggedDashboardChooser<Integer>("Sweep Cycles Chooser");
    private static final LoggedDashboardChooser<Boolean> waitingChooser =
            new LoggedDashboardChooser<Boolean>("Waiting Chooser");

    private static final Alert competitionNotSelectedAlert =
            new Alert("Competition auto is not running", Alert.AlertType.kWarning);
    private static final Alert invalidConfigAlert =
            new Alert(
                    "Auto configuration is invalid, fallback to Command.None()",
                    Alert.AlertType.kError);
    private static final Alert nullConfigAlert =
            new Alert(
                    "Auto configuration is null, fallback to Command.None()",
                    Alert.AlertType.kError);

    private static <E extends Enum<E>> void initializeChooser(
            LoggedDashboardChooser<E> chooser, E[] values, E defaultValue) {
        chooser.onChange((value) -> verifyOption());
        chooser.addDefaultOption(defaultValue.toString(), defaultValue);
        for (E e : values) {
            chooser.addOption(e.toString(), e);
        }
    }

    public static void init() {
        initializeAutoPaths();
        initializeChooser(autoChooser, AutoType.values(), AutoType.COMPETITION);
        initializeChooser(sideChooser, AutoSide.values(), AutoSide.RIGHT);
        initializeChooser(endBehaviourChooser, EndBehaviour.values(), EndBehaviour.FUEL);
        initializeChooser(sweepModeChooser, SweepMode.values(), SweepMode.LONG);

        // Initialize sweep cycles chooser (only for FAST mode)
        sweepCyclesChooser.addDefaultOption("1", 1);
        sweepCyclesChooser.addOption("2", 2);
        sweepCyclesChooser.addOption("3", 3);

        waitingChooser.addDefaultOption("Wait", true);
        waitingChooser.addOption("No Waiting", false);
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

    public static Command buildTest() {
        return AutoActions.drivePastSlope(false, true);
    }

    public static Command buildAuto() {
        AutoType selected = autoChooser.get();
        if (selected == null) {
            invalidConfigAlert.set(true);
            return Commands.none();
        }

        return switch (selected) {
            case TEST -> buildTest();
            case COMPETITION -> buildCompetition();
            case HUNT -> buildHunt();
            case SHOOT -> Commands.defer(
                    () -> shootingSuperstructure.shootWhenReady(false),
                    Collections.singleton(shooter));
        };
    }

    private static void verifyOption() {
        if (autoChooser.get() == null
                || sweepModeChooser.get() == null
                || endBehaviourChooser.get() == null
                || sideChooser.get() == null) {
            nullConfigAlert.set(true);
            SmartDashboard.putBoolean("Auto/Verified", false);
            return;
        } else {
            nullConfigAlert.set(false);
        }

        if (autoChooser.get() == AutoType.COMPETITION) {
            competitionNotSelectedAlert.set(false);
            invalidConfigAlert.set(false);
        } else {
            competitionNotSelectedAlert.set(true);
            invalidConfigAlert.set(false);
        }
        SmartDashboard.putBoolean(
                "Auto/Verified", !invalidConfigAlert.get() && !competitionNotSelectedAlert.get());
    }

    /**
     * Builds a single sweep and shoot cycle.
     *
     * @param sweepPathName The path to follow for sweeping
     * @param isLeft Whether this is the left side
     * @return Command for one sweep/shoot cycle
     */
    private static Command buildSweepShootCycle(String sweepPathName, boolean isLeft) {
        return Commands.sequence(
                Commands.deadline(followPathFile(sweepPathName, isLeft), intake()),
                drivePastSlope(isLeft, false),
                Commands.deadline(driveToShoot(isLeft), shoot().withTimeout(20.0)),
                new WaitCommand(3),
                allignToStarting(isLeft),
                Commands.runOnce(() -> {})
                        .withTimeout(0.1)
                        .deadlineFor(
                                shooterDefault(),
                                shootingSuperstructure.getIdx().runState(() -> IdxMode.OFF)));
    }

    private static Command buildCompetition() {
        if (invalidConfigAlert.get()) return Commands.none();
        boolean isLeft = sideChooser.get() == AutoSide.LEFT;
        SweepMode sweepMode = sweepModeChooser.get();

        String sweepPathName =
                switch (sweepMode) {
                    case FAST -> "quickSweepRight";
                    case LONG -> "longSweepRight";
                    case NORMAL -> "sweepRight";
                };

        // Build sweep/shoot cycles for FAST mode
        Command sweepSequence;
        if (sweepMode == SweepMode.FAST) {
            int cycles = sweepCyclesChooser.get() != null ? sweepCyclesChooser.get() : 1;
            Command[] cycleCommands = new Command[cycles];
            for (int i = 0; i < cycles; i++) {
                if (i == cycles - 1) {
                    // Last cycle: only sweep and drive, no shoot
                    cycleCommands[i] =
                            Commands.sequence(
                                    Commands.deadline(
                                            followPathFile(sweepPathName, isLeft), intake()),
                                    drivePastSlope(isLeft, false));
                } else {
                    // All other cycles: full sweep/shoot cycle
                    cycleCommands[i] = buildSweepShootCycle(sweepPathName, isLeft);
                }
            }
            sweepSequence = Commands.sequence(cycleCommands);
        } else {
            // For LONG and NORMAL modes, use original single sweep logic
            sweepSequence =
                    Commands.sequence(
                                    new WaitCommand(waitingChooser.get() ? 2 : 0),
                                    Commands.deadline(
                                            followPathFile(sweepPathName, isLeft), intake()),
                                    drivePastSlope(isLeft, false))
                            .alongWith();
        }

        return Commands.parallel(
                        // shooterDefault(),
                        // Start and run timer in parallel
                        Commands.runOnce(
                                () -> {
                                    autoTimer.reset();
                                    autoTimer.start();
                                }),
                        Commands.sequence(
                                // Initial drive past slope with zeroing
                                drivePastSlope(isLeft, true),

                                // Sweep sequence (single or multiple cycles)
                                sweepSequence,

                                // FUEL
                                Commands.parallel(
                                                shoot(),
                                                Commands.sequence(
                                                        Commands.deadline(
                                                                new WaitUntilCommand(
                                                                        () ->
                                                                                autoTimer.get()
                                                                                        >= 17),
                                                                new ConditionalCommand(
                                                                        Commands.deadline(
                                                                                allignToDepot(),
                                                                                oscillateIntakeFeed()),
                                                                        Commands.deadline(
                                                                                allignToStation(),
                                                                                oscillateIntakeFeed()),
                                                                        () -> isLeft)),
                                                        driveToShoot(isLeft)),
                                                new WaitUntilCommand(
                                                                () ->
                                                                        isLeft
                                                                                && autoChooser.get()
                                                                                        == AutoType
                                                                                                .COMPETITION
                                                                                && sweepModeChooser
                                                                                                .get()
                                                                                        == SweepMode
                                                                                                .LONG
                                                                                && endBehaviourChooser
                                                                                                .get()
                                                                                        == EndBehaviour
                                                                                                .FUEL
                                                                                && autoTimer.get()
                                                                                        >= 18)
                                                        .andThen(intake()))
                                        .onlyIf(
                                                () ->
                                                        endBehaviourChooser.get()
                                                                == EndBehaviour.FUEL),

                                // INTAKE
                                Commands.sequence(
                                                driveToShoot(isLeft),
                                                shoot().withTimeout(5.0),
                                                allignToStarting(isLeft),
                                                Commands.runOnce(() -> {})
                                                        .withTimeout(0.1)
                                                        .deadlineFor(
                                                                shooterDefault(),
                                                                shootingSuperstructure
                                                                        .getIdx()
                                                                        .runState(
                                                                                () -> IdxMode.OFF)),
                                                drivePastSlope(isLeft, false),
                                                Commands.deadline(
                                                        followPathFile("rightIntake", isLeft),
                                                        intake()))
                                        .onlyIf(
                                                () ->
                                                        endBehaviourChooser.get()
                                                                == EndBehaviour.INTAKE)))
                .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming);
        // .beforeStarting(() -> swerve.removeDefaultCommand());
    }

    private static Command buildHunt() {
        if (invalidConfigAlert.get()) return Commands.none();
        boolean isLeft = sideChooser.get() == AutoSide.LEFT;
        Command driveToNeutral =
                deadline(
                        drivePastSlope(isLeft, true),
                        defer(AutoActions::zeroEverything, Collections.emptySet()));
        Command hunt =
                deadline(followPathFile("huntRight", isLeft), intake(), shootBackWhenSuitable());
        Command driveBack = drivePastSlope(isLeft, false);
        Command driveToCorner =
                either(
                        deadline(allignToDepot(), oscillateIntakeFeed()),
                        deadline(allignToStation(), oscillateIntakeFeed()),
                        () -> isLeft);
        Command sweepToClimb =
                either(
                        deadline(driveToClimbSweepLeft(), oscillateIntakeFeed()),
                        deadline(driveToClimbSweepRight(), oscillateIntakeFeed()),
                        () -> isLeft);
        var resetWhenNeeded =
                waitUntil(() -> isLeft && DriverStation.isAutonomous() && autoTimer.get() >= 18)
                        .andThen(
                                parallel(
                                        defer(intake::zeroCommand, Collections.emptySet()),
                                        intake()));
        return sequence(
                        driveToNeutral, //
                        hunt, //
                        driveBack, //
                        parallel( //
                                shoot(),
                                sequence(driveToCorner, waitSeconds(1.5), sweepToClimb),
                                resetWhenNeeded))
                .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming);
    }

    private enum AutoType {
        COMPETITION,
        TEST,
        SHOOT,
        HUNT
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
        INTAKE,
        FUEL
    }
}
