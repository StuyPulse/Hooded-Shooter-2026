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
    private Rotation2d shootAngle;
    private Rotation2d ferryAngle;


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
        
        targetAngle = Rotation2d.kZero;
        shootAngle = Rotation2d.kZero;
        ferryAngle = Rotation2d.kZero;

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

    public void setShootAngle(double shootAngleRads) { // Use this method in commmand with the swerve, turret, and hdsr subsystems
        double shootingAngle = shootAngleRads * 1180 / Math.PI;
        if (Math.abs(shootingAngle % 360) > 40){
            shootingAngle = 40;
        } else if (Math.abs(shootingAngle % 360) < 5){
            shootingAngle = 5;
        }
        this.shootAngle = Rotation2d.fromDegrees(shootingAngle);
    }

    public void setFerryAngle(Rotation2d ferryAngle) {
        if (ferryAngle.getDegrees
        this.ferryAngle = ferryAngle;
        this.ferryAngle = ferryAngle;
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

    

    public double getShootRPM() {
        return Constants.Shooter.SHOT_RPM;
    }

    public double getFerryRPM() {
        return Constants.Shooter.FERRY_RPM; 
    }

    public boolean spunUp() {
        double diff = Math.abs(getTargetRPM() - (getState() == State.FERRY ? getFerryRPM() : getShootRPM()));
        return (diff > Settings.Shooter.shooterRpmTolerance.getAsDouble()) ? true : false;
    }

    public abstract double getFlywheelRPM();

    @Override 
    public void periodic() {
        SmartDashboard.putString("hdsr/state", getState().name());
        SmartDashboard.putNumber("hdsr/targetShootAngle", getShootAngle().getDegrees());
        SmartDashboard.putNumber("hdsr/currentAngle", getCurrentAngle().getDegrees());
    }
}
