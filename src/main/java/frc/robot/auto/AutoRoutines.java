package frc.robot.auto;

import static frc.robot.auto.AutoActions.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.Collections;
import lib.ironpulse.swerve.Swerve;

public class AutoRoutines {
    public static Swerve swerve;

    public static void init(Swerve swerve) {
        AutoRoutines.swerve = swerve;
    }

    public static Command buildRoutineWithZeroing(Command routine) {
        return Commands.parallel(routine, zeroEverything());
    }

    public static Command sweepLeftClimb() {
        return Commands.defer(
                () ->
                        Commands.sequence(
                                drivePastSlope(true, true),
                                Commands.deadline(followPathFile("sweepRight", true), intake()),
                                drivePastSlope(true, false),
                                shoot()),
                Collections.singleton(swerve));
    }

    public static Command sweepRightClimb() {
        return Commands.defer(
                () ->
                        Commands.sequence(
                                drivePastSlope(false, true),
                                Commands.deadline(followPathFile("sweepRight", false), intake()),
                                drivePastSlope(false, false),
                                shoot().alongWith(
                                                Commands.sequence(
                                                        Commands.deadline(
                                                                allignToStation(),
                                                                oscillateIntakeFeed()),
                                                        Commands.waitSeconds(1),
                                                        Commands.parallel(
                                                                oscillateIntakeFeed(),
                                                                alignToClimb(false)
                                                                        .andThen(simpleClimb()))))),
                Collections.singleton(swerve));
    }

    public static Command quickSweepRight() {
        return Commands.defer(
                () ->
                        Commands.sequence(
                                drivePastSlope(false, true),
                                Commands.deadline(
                                        followPathFile("quickSweepRight", false), intake()),
                                drivePastSlope(false, false),
                                shoot().alongWith(
                                                Commands.sequence(
                                                        Commands.deadline(
                                                                allignToStation(), intake()),
                                                        runFeed()))),
                Collections.singleton(swerve));
    }

    public static Command longSweepRight() {
        return Commands.defer(
                () ->
                        Commands.sequence(
                                drivePastSlope(false, true),
                                Commands.deadline(
                                        followPathFile("longSweepRight", false), intake()),
                                drivePastSlope(false, false),
                                shoot().alongWith(
                                                Commands.sequence(
                                                        Commands.deadline(
                                                                allignToStation(), intake())))),
                Collections.singleton(swerve));
    }

    public static Command longSweepLeft() {
        return Commands.defer(
                () ->
                        Commands.sequence(
                                drivePastSlope(true, true),
                                Commands.deadline(followPathFile("longSweepRight", true), intake()),
                                drivePastSlope(true, false),
                                shoot().alongWith(
                                                Commands.sequence(
                                                        Commands.deadline(
                                                                Commands.none(), intake())))),
                Collections.singleton(swerve));
    }

    public static Command testPath() {
        return followPathFile("testPath", false);
    }
}
