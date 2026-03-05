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
                                Commands.deadline(driveSweep(true), Intake()),
                                drivePastSlope(true, false),
                                shoot().alongWith(
                                                driveToPose(
                                                        () ->
                                                                AllianceFlipUtil.apply(
                                                                        kStationIntake)))),
                Collections.singleton(swerve));
    }

    public static Command sweepRightClimb() {
        return Commands.none();
    }

    public static Command quickSweepLeft() {
        return Commands.defer(
                () ->
                        Commands.sequence(
                                drivePastSlope(true, true),
                                Commands.deadline(
                                        driveToPose(
                                                () -> AllianceFlipUtil.apply(kQuickSweependPoseL)),
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
