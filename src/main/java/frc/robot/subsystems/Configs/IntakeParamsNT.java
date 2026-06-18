package frc.robot.subsystems.Configs;

import lib.ntext.NTParameter;

@NTParameter(tableName = "Params/Intake")
public class IntakeParamsNT {

    public static final double intakeVelRPS = 52;
    public static final double outtakeVelRPS = -50;

    public static final double slowIntakeMultiplier = 0.6;
    public static final double fastIntakeMultiplier = 1.0;
}
