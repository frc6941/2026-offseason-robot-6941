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
    private static final LoggedDashboardChooser<Boolean> revertChooser =
            new LoggedDashboardChooser<Boolean>("Revert Path Chooser");
    private static final LoggedDashboardChooser<AntiSweepBehavior> antiSweepBehavioiurChooser =
            new LoggedDashboardChooser<AntiSweepBehavior>("Anti Sweep Behaviour Chooser");
    private static double antiSweepWaitTime = 0.0;

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
        initializeChooser(
                antiSweepBehavioiurChooser, AntiSweepBehavior.values(), AntiSweepBehavior.SHOOT);
        initializeChooser(sideChooser, AutoSide.values(), AutoSide.RIGHT);
        initializeChooser(endBehaviourChooser, EndBehaviour.values(), EndBehaviour.FUEL);
        initializeChooser(sweepModeChooser, SweepMode.values(), SweepMode.LONG);

        // Initialize sweep cycles chooser (only for FAST mode)
        sweepCyclesChooser.addDefaultOption("1", 1);
        sweepCyclesChooser.addOption("2", 2);
        sweepCyclesChooser.addOption("3", 3);

        waitingChooser.addDefaultOption("Wait", true);
        waitingChooser.addOption("No Waiting", false);

        revertChooser.addDefaultOption("No Revert", false);
        revertChooser.addOption("Revert", true);

        // Initialize AntiSweep wait time as typable field
        SmartDashboard.putNumber("Auto/AntiSweep Wait Time", antiSweepWaitTime);
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
            case COMPETITION, ANTI_SWEEP_COMPETITION -> buildCompetition();
            case HUNT -> buildHunt();
            case SHOOT -> Commands.defer(
                    () -> shootingSuperstructure.shootWhenReady(false),
                    Collections.singleton(shooter));
        };
    }

    /** Returns true if current auto mode is AntiSweep with PASS behavior */
    public static boolean isAntiSweepPass() {
        return autoChooser.get() == AutoType.ANTI_SWEEP_COMPETITION
                && antiSweepBehavioiurChooser.get() == AntiSweepBehavior.PASS;
    }

    /** Returns true if current auto mode is Hunt */
    public static boolean isHunt() {
        return autoChooser.get() == AutoType.HUNT;
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
                Commands.parallel(
                        rotateToShoot(isLeft),
                        new WaitCommand(3)
                                .withTimeout(3)
                                .deadlineFor(
                                        shootingSuperstructure
                                                .shootWhenReady(false)
                                                .alongWith(oscillateIntakeFeed()))),
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
        boolean isAntiSweep = autoChooser.get() == AutoType.ANTI_SWEEP_COMPETITION;
        AntiSweepBehavior antiSweepBehavior = antiSweepBehavioiurChooser.get();
        boolean isAntiSweepPass = isAntiSweep && antiSweepBehavior == AntiSweepBehavior.PASS;

        String sweepPathName;
        switch (sweepMode) {
            case FAST:
                sweepPathName = "DoubleSweep-1";
                break;
            case LONG:
                sweepPathName = "longSweepRightNew";
                break;
            case NORMAL:
                sweepPathName = "sweepRight";
                break;
            default:
                sweepPathName = "sweepRight";
                break;
        }

        // Build sweep/shoot cycles for FAST mode
        Command sweepSequence;
        if (sweepMode == SweepMode.FAST) {
            int cycles = sweepCyclesChooser.get() != null ? sweepCyclesChooser.get() : 1;
            boolean useRevert = revertChooser.get() != null ? revertChooser.get() : false;

            // If AntiSweep PASS mode, only do first cycle (no second cycle)
            int effectiveCycles = isAntiSweepPass ? 1 : cycles;
            Command[] cycleCommands = new Command[effectiveCycles];

            for (int i = 0; i < effectiveCycles; i++) {
                // Determine path to use
                String pathToUse;
                if (isAntiSweep && i == 0) {
                    // Use AntiSweep path for first cycle if in ANTI_SWEEP_COMPETITION mode
                    pathToUse = "AntiSweep";
                } else if (cycles == 2 && i == 1 && useRevert) {
                    // Use DoubleSweep-2 for second cycle when revert=true and cycles=2
                    pathToUse = "DoubleSweep-2";
                } else if (cycles == 2 && i == 1 && !useRevert) {
                    // Use DoubleSweep-2 for second cycle when revert=true and cycles=2
                    pathToUse = "DoubleSweep-2-NoRevert";
                } else {
                    // Use default sweep path
                    pathToUse = sweepPathName;
                }

                // If AntiSweep PASS mode on first cycle, just drive path without shooting
                if (isAntiSweepPass && i == 0) {
                    cycleCommands[i] =
                            Commands.sequence(
                                    Commands.deadline(
                                            followPathFile(pathToUse, isLeft), intake(), shoot()));
                } else {
                    // All cycles: full sweep/shoot cycle
                    cycleCommands[i] = buildSweepShootCycle(pathToUse, isLeft);
                }
            }
            sweepSequence = Commands.sequence(cycleCommands);
        } else {
            // For LONG and NORMAL modes, use original single sweep logic
            String pathToUse = isAntiSweep ? "AntiSweep" : sweepPathName;
            sweepSequence =
                    Commands.sequence(
                            new WaitCommand(waitingChooser.get() ? 2 : 0),
                            Commands.deadline(followPathFile(pathToUse, isLeft), intake()));
        }

        return Commands.parallel(
                // shooterDefault(),
                Commands.sequence(
                                Commands.runOnce(() -> {})
                                        .withTimeout(0.1)
                                        .deadlineFor(shootingSuperstructure.runFrame()),

                                // Wait before starting (only for AntiSweep)
                                new WaitCommand(
                                        isAntiSweep
                                                ? SmartDashboard.getNumber(
                                                        "Auto/AntiSweep Wait Time", 0.0)
                                                : 0.0),

                                // Start timer after wait command
                                Commands.runOnce(
                                        () -> {
                                            autoTimer.reset();
                                            autoTimer.start();
                                        }),

                                // Sweep sequence (single or multiple cycles)
                                sweepSequence,

                                // FUEL or INTAKE based on end behaviour
                                Commands.either(
                                        // FUEL path
                                        Commands.parallel(
                                                Commands.sequence(
                                                        new ConditionalCommand(
                                                                Commands.deadline(
                                                                        followPathFile(
                                                                                "depot", false),
                                                                        intake(),
                                                                        shoot()),
                                                                // Skip station if FAST mode
                                                                // (DoubleSweep)
                                                                (sweepMode == SweepMode.FAST
                                                                                && sweepCyclesChooser
                                                                                                .get()
                                                                                        == 2
                                                                        ? Commands.none()
                                                                        : Commands.deadline(
                                                                                followPathFile(
                                                                                        "station",
                                                                                        false),
                                                                                oscillateIntakeFeed(),
                                                                                shoot())),
                                                                () -> isLeft),
                                                        new ConditionalCommand(
                                                                new WaitUntilCommand(
                                                                        () ->
                                                                                autoTimer.get()
                                                                                        >= 20),
                                                                new WaitUntilCommand(
                                                                        () ->
                                                                                autoTimer.get()
                                                                                        >= 19),
                                                                () -> isLeft),
                                                        driveToShoot(isLeft)
                                                                .onlyIf(() -> isLeft == false),
                                                        new WaitUntilCommand(
                                                                        () ->
                                                                                isLeft
                                                                                        && autoChooser
                                                                                                        .get()
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
                                                                                        && autoTimer
                                                                                                        .get()
                                                                                                >= 18)
                                                                .andThen(intake()))),
                                        // INTAKE path
                                        Commands.parallel(
                                                Commands.sequence(
                                                        new ConditionalCommand(
                                                                new WaitUntilCommand(
                                                                        () ->
                                                                                autoTimer.get()
                                                                                        >= 18),
                                                                new WaitUntilCommand(
                                                                        () ->
                                                                                autoTimer.get()
                                                                                        >= 18),
                                                                () -> isLeft),
                                                        Commands.runOnce(() -> {})
                                                                .withTimeout(0.1)
                                                                .deadlineFor(
                                                                        shooterDefault(),
                                                                        shootingSuperstructure
                                                                                .getIdx()
                                                                                .runState(
                                                                                        () ->
                                                                                                IdxMode
                                                                                                        .OFF)),
                                                        new ConditionalCommand(
                                                                followPathFile("leftIntake", false),
                                                                followPathFile(
                                                                        "rightIntake", false),
                                                                () -> isLeft))),
                                        () -> endBehaviourChooser.get() == EndBehaviour.FUEL))
                        .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming));
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
                deadline(
                        followPathFile("newhuntRight", isLeft),
                        intake(),
                        new WaitCommand(10).withTimeout(10).deadlineFor(shootBackWhenSuitable()));
        Command driveBack = drivePastSlope(isLeft, false);
        Command driveToCorner =
                either(
                        deadline(followPathFile("depot", false), intake()),
                        deadline(followPathFile("station", false), oscillateIntakeFeed()),
                        () -> isLeft);
        Command sweepToClimb =
                either(
                        deadline(followPathFile("huntDepot", false), oscillateIntakeFeed()),
                        deadline(followPathFile("huntStation", false), oscillateIntakeFeed()),
                        () -> isLeft);
        var resetWhenNeeded =
                waitUntil(() -> isLeft && DriverStation.isAutonomous() && autoTimer.get() >= 18)
                        .andThen(
                                parallel(
                                        defer(intake::zeroCommand, Collections.emptySet()),
                                        intake()));
        return sequence(
                        // driveToNeutral, //
                        hunt, //
                        // driveBack, //
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
        HUNT,
        ANTI_SWEEP_COMPETITION
    }

    private enum AntiSweepBehavior {
        SHOOT,
        PASS
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
