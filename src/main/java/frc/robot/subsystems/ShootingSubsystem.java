package frc.robot.subsystems;

import lib.ironpulse.io.MotorInputsAutoLogged;
import lib.ironpulse.io.MotorIO;
import lib.ironpulse.subsystem.velocity.VelocityMotorSubsystem;
import lombok.Getter;

public class ShootingSubsystem{

    @Getter private final VelocityMotorSubsystem<MotorInputsAutoLogged, MotorIO> shooter;
    @Getter private final SpindexerSubsystem idx;

    public class ShootingSubsystem(
            VelocityMotorSubsystem<MotorInputsAutoLogged, MotorIO> shooter,
            SpindexerSubsystem idx
    ){
        this.shooter = shooter;
        this.idx = idx;
    }






}
