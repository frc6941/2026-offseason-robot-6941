package frc.robot.auto;

import static frc.robot.auto.AutoActions.*;
import static frc.robot.auto.AutoRoutines.*;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
          "Auto configuration is invalid, fallback to Command.None()", Alert.AlertType.kError);
  private static final Alert nullConfigAlert =
      new Alert("Auto configuration is null, fallback to Command.None()", Alert.AlertType.kError);

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
        throw new IllegalArgumentException("Failed to parse path file: " + file.getName(), e);
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

    AutoType selected = autoChooser.get();
    if (selected == null) {
      invalidConfigAlert.set(true);
      return Commands.none();
    }

    return switch (selected) {
      case TEST -> buildTest();
      case COMPETITION -> buildCompetition();
      case SHOOT ->
          Commands.defer(
              () -> shootingSuperstructure.shootWhenReady(false), Collections.singleton(shooter));
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
      invalidConfigAlert.set(
          sweepModeChooser.get() == SweepMode.LONG
              && endBehaviourChooser.get() != EndBehaviour.FUEL);
    } else {
      competitionNotSelectedAlert.set(true);
      invalidConfigAlert.set(false);
    }
    SmartDashboard.putBoolean(
        "Auto/Verified", !invalidConfigAlert.get() && !competitionNotSelectedAlert.get());
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
    return Commands.parallel(
            // shooterDefault(),
            Commands.sequence(
                // Sweep
                Commands.deadline(
                    drivePastSlope(isLeft, true),
                    Commands.defer(AutoActions::zeroEverything, Collections.emptySet())),
                Commands.deadline(followPathFile(sweepPathName, isLeft), intake()),
                drivePastSlope(isLeft, false),

                // FUEL
                Commands.parallel(
                        shoot(),
                        new ConditionalCommand(
                            Commands.deadline(allignToDepot(), oscillateIntakeFeed()),
                            Commands.deadline(allignToStation(), oscillateIntakeFeed()),
                            () -> isLeft))
                    .onlyIf(() -> endBehaviourChooser.get() == EndBehaviour.FUEL),

                // CLIMB
                Commands.sequence(
                        Commands.deadline(
                            allignToClimb(isLeft),
                            Commands.parallel(
                                oscillateIntakeFeed().withTimeout(20),
                                climbUp(),
                                shoot().withTimeout(20))),
                        climbed().alongWith(shoot()))
                    .onlyIf(() -> endBehaviourChooser.get() == EndBehaviour.CLIMB)),
            Commands.sequence(
                    Commands.waitUntil(()-> DriverStation.isAutonomous() && DriverStation.getMatchTime()<=2),
            Commands.parallel(intake.zeroCommand(), intake()))
        .onlyIf(
            () ->
                isLeft
                    && autoChooser.get() == AutoType.COMPETITION
                    && sweepModeChooser.get() == SweepMode.LONG
                    && endBehaviourChooser.get() == EndBehaviour.FUEL)
        .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming);
    // .beforeStarting(() -> swerve.removeDefaultCommand());
  }

  private enum AutoType {
    COMPETITION,
    TEST,
    SHOOT
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
    FUEL
  }
}
