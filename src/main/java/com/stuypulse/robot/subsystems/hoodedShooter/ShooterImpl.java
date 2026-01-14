package com.stuypulse.robot.subsystems.hoodedShooter;

import com.stuypulse.robot.constants.Motors;

import com.ctre.phoenix6.hardware.TalonFX;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;


public class ShooterImpl extends Shooter{

    private final TalonFX leader;
    private final TalonFX follower;
    

    protected ShooterImpl() {
        leader = new TalonFX(Ports.HDSR.LEADER);
        leader.getConfigurator().apply(Motors.Shooter.configs);

        follower = new TalonFX(Ports.HDSR.FOLLOWER);
        follower.getConfigurator().apply(Motors.Shooter.configs);
    }

    @Override
    public double getFlywheelRPM() {
        return leader.getVelocity().getValueAsDouble() * 60;
        
    }

    @Override
    public double getShootRPM() {
        return 0;
    }

    @Override 
    public double getFerryRPM() {
        return 0; 
    }

    @Override
    public boolean spunUp() {
        return (Math.abs(getTargetRPM() - getFerryRPM()) > Settings.Shooter.shooterRpmTollerance.getAsDouble()) ? true : false;
    }


    @Override
    public void periodic() {
        leader.setControl(new VelocityVoltage(getTargetRPM() / 60));
    }


}
