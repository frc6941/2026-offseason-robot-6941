package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class AutoRoutines {

    public static Command sweepLeftClimb() {
        return Commands.sequence(
                AutoActions.drivePastSlope(true, true),
                Commands.deadline(AutoActions.driveSweep(true), AutoActions.Intake()),
                AutoActions.drivePastSlope(true, false),
                AutoActions.shoot()
                        .alongWith(AutoActions.driveToPose(AutoActions.kStationIntake)));
    }

    public static Command sweepRightClimb() {
        return Commands.none();
    }
}
