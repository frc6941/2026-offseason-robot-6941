package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.littletonrobotics.junction.Logger;

public class AutoRoutines {
    public static Command driveToSweepStart(boolean isLeft) {
        Command drivePastSlope = AutoActions.drivePastSlopeCommand(isLeft, true);
        Command driveToSweepPath = AutoActions.driveToSweepPathCommand(isLeft);

        return Commands.sequence(
                        drivePastSlope, driveToSweepPath)
                .alongWith(
                        Commands.runOnce(
                                () -> {
                                    Logger.recordOutput(
                                            "Temp/hasCrossedBump",
                                            AutoActions.hasCrossedBump(true));
                                    Logger.recordOutput(
                                            "Temp/pitchStable", AutoActions.isPitchStable());
                                }));
    }
}  
