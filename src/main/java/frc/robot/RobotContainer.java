// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.SwerveConstants;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.IntakePivotSubsystem;
import lib.ironpulse.io.MotorIOTalonFX;
import lib.ironpulse.io.MotorInputsAutoLogged;
import lib.ironpulse.subsystem.ServoMotorSubsystem;
import lib.ironpulse.subsystem.SubsystemConfig;
import lib.ironpulse.swerve.Swerve;
import lib.ironpulse.swerve.sim.ImuIOSim;
import lib.ironpulse.swerve.sim.SwerveModuleIOSimpleSim;
import lib.ironpulse.swerve.sjtu6.ImuIOPigeon;
import lib.ironpulse.swerve.sjtu6.SwerveModuleIOSJTU6;

public class RobotContainer {
  private Swerve swerve;
  private final IntakePivotSubsystem intakePivot = new IntakePivotSubsystem();
  private final FlywheelSubsystem flywheelSubsystem = new FlywheelSubsystem();
  private final CommandXboxController driver = new CommandXboxController(0);
  public RobotContainer() {
    if (RobotBase.isReal()){
      swerve = new Swerve(
        SwerveConstants.kRealConfig,
        new ImuIOPigeon(SwerveConstants.kRealConfig),
        new SwerveModuleIOSJTU6(SwerveConstants.kRealConfig, 0),
        new SwerveModuleIOSJTU6(SwerveConstants.kRealConfig, 1),
        new SwerveModuleIOSJTU6(SwerveConstants.kRealConfig, 2),
        new SwerveModuleIOSJTU6(SwerveConstants.kRealConfig, 3));
    } else {
      swerve = new Swerve(
        SwerveConstants.kSimConfig,
        new ImuIOSim(),
        new SwerveModuleIOSimpleSim(SwerveConstants.kSimConfig, 0),
        new SwerveModuleIOSimpleSim(SwerveConstants.kSimConfig, 1),
        new SwerveModuleIOSimpleSim(SwerveConstants.kSimConfig, 2),
        new SwerveModuleIOSimpleSim(SwerveConstants.kSimConfig, 3));
    }
    configureBindings();
  }

  public void robotPeriodic() {
    //reserved for vision fusion
  }

  private void configureBindings() {
  
    // driver.y().onTrue(Commands.runOnce(()-> intakePivot.setNeutralMode(true)));
    driver.y().onTrue(Commands.runOnce(()-> intakePivot.setMotionMagicSetpoint(Degrees.of(40.0))));
    driver.b().whileTrue(Commands.runOnce(()-> flywheelSubsystem.setVelocitySetpoint(1000)));
    driver.a().whileTrue(Commands.runOnce(()-> flywheelSubsystem.setInputVoltage(-10)));
    
    
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
