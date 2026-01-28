package com.stuypulse.robot.subsystems.hdsr;

import com.stuypulse.robot.constants.Constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class HDSR extends SubsystemBase{
    private static HDSR instance;
    private State state = State.STOW;

    static {
        instance = new HDSRImpl();
    }
    
    public static HDSR getInstance(){
        return instance;
    }

    private Rotation2d targetAngle;
    private Rotation2d shootAngle;
    private Rotation2d ferryAngle;

    public enum State {
        STOW,
        FERRY,
        SHOOT;
    }

    public abstract Rotation2d getCurrentAngle();

    public State getState(){
        return state;
    }

    public void setState(State state){
        this.state = state;
        setTargetAngle();
    }
    
    public Rotation2d getShootAngle() {
        return shootAngle;
    }

    public Rotation2d getFerryAngle() {
        return ferryAngle;
    }

    public void setAngle(Rotation2d angle) { // Debugging
        targetAngle = angle;
    }

    public void setShootAngle(Rotation2d shootAngle) { // Use this method in commmand with the swerve, turret, and hdsr subsystems
        this.shootAngle = shootAngle;
        state = State.SHOOT;
    }

    public void setFerryAngle(Rotation2d ferryAngle) {
        this.ferryAngle = ferryAngle;
        state = State.FERRY;
    }
 
    public void setTargetAngle() {
        switch (state) {
            case STOW -> targetAngle = Constants.Hood.MIN_ANGLE;
            case SHOOT -> targetAngle = getShootAngle();
            case FERRY -> targetAngle = getFerryAngle();
        };
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

    public abstract double getFerryRPM();
    public abstract double getShootRPM();
    public abstract double getFlywheelRPM();
    public abstract boolean spunUp();

}
