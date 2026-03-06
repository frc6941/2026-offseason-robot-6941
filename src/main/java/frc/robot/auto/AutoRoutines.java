package frc.robot.auto;

import static frc.robot.auto.AutoActions.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.Collections;
import lib.ironpulse.swerve.Swerve;
import lib.ironpulse.utils.AllianceFlipUtil;

public class AutoRoutines {
    public static Swerve swerve;

    public static void init(Swerve swerve) {
        AutoRoutines.swerve = swerve;
    }

    public static Command sweepLeftClimb() {
        return Commands.defer(
                () ->
                        Commands.sequence(
                                drivePastSlope(true, true),
                                Commands.deadline(followPathFile("sweepRight",true), Intake()),
                                drivePastSlope(true, false),
                                shoot()),
                Collections.singleton(swerve));
    }

    public static Command sweepRightClimb() {
        return Commands.defer(
                () ->
                        Commands.sequence(
                                drivePastSlope(false, true),
                                Commands.deadline(followPathFile("sweepRight",false), Intake()),
                                drivePastSlope(false, false),
                                shoot().alongWith(
                                        Commands.sequence(
                                                Commands.deadline(
                                                        allignToStation(), Intake()),
                                                Commands.waitSeconds(1),
                                                allignToClimb(false)).alongWith(
                                                        Commands.waitSeconds(0.5).andThen(
                                                                runFeed()
                                                        )
                                                ))),
                Collections.singleton(swerve));
    }

    public static Command quickSweepLeft() {
        return Commands.defer(
                () ->
                        Commands.sequence(
                                drivePastSlope(true, true),
                                Commands.deadline(
                                        driveToPose(
                                                () -> AllianceFlipUtil.apply(kQuickSweepEndPoseL)),
                                        Intake()),
                                drivePastSlope(true, false)),
                Collections.singleton(swerve));
    }

    public static Command quickSweepRight() {
        return Commands.defer(
                () ->
                        Commands.sequence(
                                drivePastSlope(false, true),
                                Commands.deadline(
                                        driveToPose(AllianceFlipUtil.apply(kQuickSweepEndPoseR)),
                                        Intake()),
                                drivePastSlope(false, false),
                                shoot().alongWith(
                                                Commands.sequence(
                                                        Commands.deadline(
                                                                allignToStation(), Intake()),
                                                        Commands.waitSeconds(2.5),
                                                        allignToClimb(false)))),
                Collections.singleton(swerve));
    }
}
