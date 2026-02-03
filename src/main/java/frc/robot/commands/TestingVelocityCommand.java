package frc.robot.commands;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.SerializationFeature;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SerialSubsystem.SerialSubsystem;
import lib.ironpulse.io.MotorIO;
import lib.ironpulse.io.MotorInputsAutoLogged;
import lib.ironpulse.subsystem.velocity.VelocityMotorSubsystem;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class TestingVelocityCommand extends Command {
    private final VelocityMotorSubsystem<MotorInputsAutoLogged, MotorIO> flywheel, upperRoller;
    private final SerialSubsystem arduino;
    
    private double requiredVelocity = 0.0;
    private boolean hasRecordedCurrentSpeed = false;
    private final List<TestResult> testResults = new ArrayList<>();
    private final ObjectMapper objectMapper = new ObjectMapper();
    private final File outputFile;
    private boolean hasSavedResults = false;
    
    private Command flywheelCommand;
    private Command upperRollerCommand;
    private boolean isInitialized = false;

    private static record TestResult(double linearVel,double shooterRPS) {}

    public TestingVelocityCommand(
            VelocityMotorSubsystem<MotorInputsAutoLogged, MotorIO> flywheel,
            VelocityMotorSubsystem<MotorInputsAutoLogged, MotorIO> upperRoller,
            SerialSubsystem arduino) {
        this.flywheel = flywheel;
        this.upperRoller = upperRoller;
        this.arduino = arduino;
        
        objectMapper.enable(SerializationFeature.INDENT_OUTPUT);
        
        String homePath = System.getProperty("user.home");
        outputFile = new File(homePath, "velocity_test_results.json");
        
    }

    @Override
    public void initialize() {
        requiredVelocity = 0.0;
        hasRecordedCurrentSpeed = false;
        hasSavedResults = false;
        isInitialized = false;
        testResults.clear();
        
        System.out.println("Saving to "+outputFile.getAbsolutePath());
    }

    @Override
    public void execute() {
        if (!isInitialized) {
            requiredVelocity = 5.0;
            upperRollerCommand = upperRoller.runVelocity(RotationsPerSecond.of(30));
            upperRollerCommand.schedule();
            
            flywheelCommand = flywheel.runVelocity(RotationsPerSecond.of(requiredVelocity));
            flywheelCommand.schedule();
            
            isInitialized = true;
            System.out.println(requiredVelocity + " RPS");
            return;
        }
        if (arduino.isSpeedChanged() && !hasRecordedCurrentSpeed && requiredVelocity > 0) {
            AngularVelocity flywheelVelocity = flywheel.getVelocity();
            double shooterRPS = flywheelVelocity.in(RotationsPerSecond);
            double linearVel = arduino.getSpeed();
            testResults.add(new TestResult(linearVel, shooterRPS));
            
            System.out.printf("linearVel=%.2f, shooterRPS=%.2f, target %.2f RPS%n", 
                linearVel, shooterRPS, requiredVelocity);
            
            hasRecordedCurrentSpeed = true;
            
            if (requiredVelocity <= 110.0) {
                requiredVelocity += 5.0;
                hasRecordedCurrentSpeed = false;
                
                flywheelCommand.cancel();
                flywheelCommand = flywheel.runVelocity(RotationsPerSecond.of(requiredVelocity));
                flywheelCommand.schedule();
                
                System.out.println("to: " + requiredVelocity + " RPS");
            } else {
                saveResultsToFile();
                hasSavedResults = true;
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (flywheelCommand != null) {
            flywheelCommand.cancel();
        }
        if (upperRollerCommand != null) {
            upperRollerCommand.cancel();
        }
        
        flywheel.runStop().schedule();
        upperRoller.runStop().schedule();
        
        if (!testResults.isEmpty() && !hasSavedResults) {
            saveResultsToFile();
        }
    }

    @Override
    public boolean isFinished() {
        return requiredVelocity > 110.0;
    }

    private void saveResultsToFile() {
        try {
            File parentDir = outputFile.getParentFile();
            if (parentDir != null && !parentDir.exists()) {
                parentDir.mkdirs();
            }
            
            objectMapper.writeValue(outputFile, testResults);
            System.out.println("Successfully saved " + testResults.size() + " test results to JSON file.");
            
        } catch (IOException e) {
            System.err.println("Failed to save test results to JSON file:");
            e.printStackTrace();
        }
    }
}