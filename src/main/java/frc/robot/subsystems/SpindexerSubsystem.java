package frc.robot.subsystems;

import lib.ironpulse.io.MotorIO;
import lib.ironpulse.io.MotorInputsAutoLogged;
import lib.ironpulse.subsystem.SubsystemConfig;
import lib.ironpulse.subsystem.velocity.VelocityMotorSubsystem;
import lib.ironpulse.subsystem.velocity.VelocityParamSources;

public class SpindexerSubsystem extends VelocityMotorSubsystem<MotorInputsAutoLogged, MotorIO> {
    public SpindexerSubsystem(
            SubsystemConfig config,
            MotorInputsAutoLogged inputs,
            MotorIO io,
            VelocityParamSources params) {
        super(config, inputs, io, params);
    }
}
