// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.*;
import frc.robot.subsystems.shooter.ShooterConfig;
import frc.robot.subsystems.shooter.ShootingParametersTable;
import lib.ironpulse.utils.PhoenixUtils;
import lib.ntext.NTParameterRegistry;
import org.littletonrobotics.junction.Logger;

public class RobotContainer {
    private final CommandXboxController driver = new CommandXboxController(0);
    private final ShootingParametersTable shootingParametersTable = new ShootingParametersTable();
    private final ShooterConfig shooterConfig = new ShooterConfig();

    public RobotContainer() {

        if (Logger.hasReplaySource()) {

        } else if (RobotBase.isReal()) {

        } else {

        }
        configureBindings();
    }

    public void robotPeriodic() {
        PhoenixUtils.refreshAll();
        NTParameterRegistry.refresh();
        shootingParametersTable.updateFromNT();
    }

    private void configureBindings() {}

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
