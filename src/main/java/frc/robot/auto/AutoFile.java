package frc.robot.auto;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.io.File;
import java.io.IOException;
import java.util.HashMap;
import java.util.Map;
import lombok.Getter;
import org.json.simple.parser.ParseException;
import static frc.robot.auto.AutoRoutines.*;

public class AutoFile {
    private final Map<String, PathPlannerPath> autoPaths = new HashMap<>();
    @Getter private final SendableChooser<Command> autoChooser = new SendableChooser<>();
    private AutoActions autoActions;

    public AutoFile() {
        initializeAutoPaths();
        initializeAutoChooser();
    }

    private void initializeAutoChooser() {
        autoChooser.setDefaultOption("None", Commands.none());
        autoChooser.addOption("Test", buildTest());
        autoChooser.addOption("quickSweepRight", buildRoutineWithZeroing(quickSweepRight()));
        autoChooser.addOption("quickSweepLeft", buildRoutineWithZeroing(longSweepRight()));
        autoChooser.addOption("sweepLeftClimb", buildRoutineWithZeroing(sweepLeftClimb()));
    }


    private void initializeAutoPaths() {
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

    private PathPlannerPath getAutoPath(String path) {
        assert autoPaths.containsKey(path);
        return autoPaths.get(path);
    }

    public Command buildTest() {
        return AutoActions.drivePastSlope(false, true);
    }
}
