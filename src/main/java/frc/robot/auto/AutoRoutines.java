package frc.robot.auto;

import static frc.robot.auto.AutoActions.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.ShootingSubsystem.SpindexerSubsystem;
import java.util.Collections;
import java.util.Set;
import lib.ironpulse.subsystem.position.PositionMotorSubsystem;
import lib.ironpulse.subsystem.velocity.VelocityMotorSubsystem;
import lib.ironpulse.swerve.Swerve;

public class AutoRoutines {
    public static Swerve swerve;
    public static VelocityMotorSubsystem shooter;
    public static SpindexerSubsystem spindexer;
    public static PositionMotorSubsystem intakerExtension;

    public static void init(
            Swerve swerve,
            VelocityMotorSubsystem shooter,
            SpindexerSubsystem spindexer,
            PositionMotorSubsystem intakerExtension) {
        AutoRoutines.swerve = swerve;
        AutoRoutines.shooter = shooter;
        AutoRoutines.spindexer = spindexer;
        AutoRoutines.intakerExtension = intakerExtension;
    }

    public static Command buildRoutineWithZeroing(Command routine) {
        return Commands.parallel(routine, zeroEverything());
    }

    public static Command testRoutine() {
        return Commands.sequence(resetOnPose(AutoActions.kTestA), testPath());
    }

    // LEFT side routines
    public static Command leftFastClimb() {
        return Commands.defer(
                () ->
                        Commands.sequence(
                                Commands.deadline(drivePastSlope(false, true), zeroEverything()),
                                Commands.deadline(
                                        followPathFile("quickSweepRight", true), intake()),
                                drivePastSlope(true, false),
                                Commands.deadline(
                                        allignToClimb(true),
                                        Commands.parallel(
                                                oscillateIntakeFeed().withTimeout(20.0),
                                                climbUp(),
                                                shoot().withTimeout(20.0))),
                                climbed().alongWith(shoot())),
                Collections.singleton(swerve));
    }

    public static Command leftFastFuel() {
        return Commands.defer(
                () ->
                        Commands.sequence(
                                Commands.deadline(drivePastSlope(false, true), zeroEverything()),
                                Commands.deadline(
                                        followPathFile("quickSweepRight", true), intake()),
                                drivePastSlope(true, false),
                                Commands.parallel(
                                        shoot(),
                                        Commands.deadline(allignToDepot(), oscillateIntakeFeed()))),
                Collections.singleton(swerve));
    }

    public static Command leftNormalClimb() {
        return Commands.defer(
                () ->
                        Commands.sequence(
                                Commands.deadline(drivePastSlope(false, true), zeroEverything()),
                                Commands.deadline(followPathFile("sweepRight", true), intake()),
                                drivePastSlope(true, false),
                                Commands.deadline(
                                        allignToClimb(true),
                                        Commands.parallel(
                                                oscillateIntakeFeed().withTimeout(20.0),
                                                climbUp(),
                                                shoot().withTimeout(20.0))),
                                climbed().alongWith(shoot())),
                Collections.singleton(swerve));
    }

    public static Command leftNormalFuel() {
        return Commands.defer(
                () ->
                        Commands.sequence(
                                Commands.deadline(drivePastSlope(false, true), zeroEverything()),
                                Commands.deadline(followPathFile("sweepRight", true), intake()),
                                drivePastSlope(true, false),
                                Commands.parallel(
                                        shoot(),
                                        Commands.deadline(allignToDepot(), oscillateIntakeFeed()))),
                Collections.singleton(swerve));
    }

    public static Command leftLongFuel() {
        return Commands.defer(
                () ->
                        Commands.sequence(
                                Commands.deadline(drivePastSlope(false, true), zeroEverything()),
                                Commands.deadline(followPathFile("longSweepRight", true), intake()),
                                drivePastSlope(true, false),
                                Commands.parallel(
                                        shoot(),
                                        Commands.deadline(allignToDepot(), oscillateIntakeFeed()))),
                Collections.singleton(swerve));
    }

    // RIGHT side routines
    public static Command rightFastClimb() {
        return Commands.defer(
                () ->
                        Commands.sequence(
                                Commands.deadline(drivePastSlope(false, true), zeroEverything()),
                                Commands.deadline(
                                        followPathFile("quickSweepRight", false), intake()),
                                drivePastSlope(false, false),
                                Commands.deadline(
                                        allignToClimb(false),
                                        Commands.parallel(
                                                oscillateIntakeFeed().withTimeout(20.0),
                                                climbUp(),
                                                shoot().withTimeout(20.0))),
                                climbed().alongWith(shoot())),
                Collections.singleton(swerve));
    }

    public static Command rightFastFuel() {
        return Commands.defer(
                () ->
                        Commands.sequence(
                                Commands.deadline(drivePastSlope(false, true), zeroEverything()),
                                Commands.deadline(
                                        followPathFile("quickSweepRight", false), intake()),
                                drivePastSlope(false, false),
                                Commands.parallel(
                                        shoot(),
                                        Commands.deadline(
                                                allignToStation(), oscillateIntakeFeed()))),
                Collections.singleton(swerve));
    }

    public static Command rightNormalClimb() {
        return Commands.defer(
                () ->
                        Commands.sequence(
                                Commands.deadline(drivePastSlope(false, true), zeroEverything()),
                                Commands.deadline(followPathFile("sweepRight", false), intake()),
                                drivePastSlope(false, false),
                                Commands.deadline(
                                        allignToClimb(false),
                                        Commands.parallel(
                                                oscillateIntakeFeed().withTimeout(20.0),
                                                climbUp(),
                                                shoot().withTimeout(20.0))),
                                climbed().alongWith(shoot())),
                Collections.singleton(swerve));
    }

    public static Command rightNormalFuel() {
        return Commands.defer(
                () ->
                        Commands.sequence(
                                Commands.deadline(drivePastSlope(false, true), zeroEverything()),
                                Commands.deadline(followPathFile("sweepRight", false), intake()),
                                drivePastSlope(false, false),
                                Commands.parallel(
                                        shoot(),
                                        Commands.deadline(
                                                allignToStation(), oscillateIntakeFeed()))),
                Collections.singleton(swerve));
    }

    public static Command rightLongFuel() {

        return Commands.defer(
                () ->
                        Commands.parallel(
                                shooterDefault(),
                                Commands.sequence(
                                        Commands.deadline(
                                                drivePastSlope(false, true), zeroEverything()),
                                        Commands.deadline(
                                                followPathFile("quickSweepRight", false), intake()),
                                        drivePastSlope(false, false),
                                        Commands.parallel(
                                                AutoActions.shoot(),
                                                Commands.deadline(
                                                        allignToStation(),
                                                        oscillateIntakeFeed())))),
                Set.of(swerve, shooter, spindexer));
    }

    public static Command shootAndClimb() {
        return shoot().alongWith(
                        Commands.sequence(
                                Commands.deadline(allignToStation(), oscillateIntakeFeed()),
                                Commands.waitSeconds(1),
                                Commands.parallel(oscillateIntakeFeed(), allignToClimb(false))));
    }
}
