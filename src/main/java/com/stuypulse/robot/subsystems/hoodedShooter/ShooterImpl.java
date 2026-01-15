package com.stuypulse.robot.subsystems.hoodedShooter;

import com.stuypulse.robot.constants.Constants;
import com.stuypulse.robot.constants.Motors;

import com.ctre.phoenix6.hardware.TalonFX;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.ctre.phoenix6.controls.VelocityVoltage;


public class ShooterImpl extends Shooter{

    private final TalonFX leader;
    private final TalonFX follower;
    

    protected ShooterImpl() {
        leader = new TalonFX(Ports.HDSR.LEADER);
        Motors.Hood.configs.configure(leader);

        follower = new TalonFX(Ports.HDSR.FOLLOWER);
        Motors.Hood.configs.configure(follower);
    }

    @Override
    public double getFlywheelRPM() {
        return leader.getVelocity().getValueAsDouble() * 60;
        
    }

    @Override
    public double getShootRPM() {
        return Constants.Shooter.SHOT_RPM;
    }

    @Override 
    public double getFerryRPM() {
        return Constants.Shooter.FERRY_RPM; 
    }

    @Override
    public boolean spunUp() {
        return (Math.abs(getTargetRPM() - getFerryRPM()) > Settings.Shooter.shooterRpmTolerance.getAsDouble()) ? true : false;
    }


    @Override
    public void periodic() {
        leader.setControl(new VelocityVoltage(getTargetRPM() / 60));
        follower.setControl(new VelocityVoltage(getTargetRPM() / 60));
    }

}
