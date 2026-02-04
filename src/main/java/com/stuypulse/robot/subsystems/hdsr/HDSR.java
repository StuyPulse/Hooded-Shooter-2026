package com.stuypulse.robot.subsystems.hdsr;

import com.stuypulse.robot.constants.Constants;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.Robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class HDSR extends SubsystemBase{
    private static final HDSR instance;
    private State state;

    private Rotation2d targetAngle;


    static {
        if (Robot.isReal())
            instance = new HDSRImpl();
        else
            instance = new HDSRSim();
    }
    
    public static HDSR getInstance(){
        return instance;
    }
    
    public enum State {
        STOW,
        FERRY,
        SHOOT;
    }

    public HDSR() {
        state = State.STOW;
        
        targetAngle = Constants.HDSR.MIN_ANGLE;

    }

    public abstract double getCurrentRPS();
    public abstract Rotation2d getCurrentAngle();

    public State getState(){
        return state;
    }

    public void setState(State state){
        this.state = state;
    }

    public void setTargetAngle(Rotation2d angle) {
        targetAngle = angle;
    }
 
    public Rotation2d getTargetAngle() {
        return targetAngle;
    }

    public double getTargetRPM() {
        return switch(state) {
            case STOW -> 0;
            case FERRY -> getFerryRPM();
            case SHOOT -> getShootRPM();
        };
    }


    public double getShootRPM() {
        return Constants.HDSR.SHOT_RPM;
    }

    public double getFerryRPM() {
        return Constants.HDSR.FERRY_RPM; 
    }

    public boolean spunUp() {
        double diff = Math.abs(getTargetRPM() - (getState() == State.FERRY ? getFerryRPM() : getShootRPM()));
        return (diff > Settings.Shooter.shooterRpmTolerance.getAsDouble()) ? true : false;
    }

    @Override 
    public void periodic() {
        SmartDashboard.putString("hdsr/state", getState().name());
        SmartDashboard.putNumber("hdsr/targetAngle", getTargetAngle().getDegrees());
        SmartDashboard.putNumber("hdsr/currentAngle", getCurrentAngle().getDegrees());
        SmartDashboard.putNumber("hdsr/currentRPM", getCurrentRPS() * 60);
        SmartDashboard.putNumber("hdsr/targetRPM", getTargetRPM());
    }
}
