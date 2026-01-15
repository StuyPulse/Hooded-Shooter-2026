package com.stuypulse.robot.subsystems.hoodedShooter;
import com.stuypulse.robot.constants.Constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Hood extends SubsystemBase {
    private static final Hood instance;
    private Hoodstate hoodstate;

    static {
        instance = new HoodImpl();
    }

    public static Hood getInstance() {
        return instance;
    }

    private Rotation2d targetAngle;
    private Rotation2d shootAngle;
    private Rotation2d ferryAngle;

    protected Hood() {
        hoodstate = Hoodstate.STOW;
    }

    public enum Hoodstate {
        STOW,
        SHOOT,
        FERRY;
    }

    public abstract Rotation2d getCurrentAngle();
    
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
    }

    public void setFerryAngle(Rotation2d ferryAngle) {
        this.ferryAngle = ferryAngle;
    }
 
    public void setTargetAngle() {
        switch (getHoodState()) {
            case STOW -> targetAngle = Constants.Hood.MIN_ANGLE;
            case SHOOT -> targetAngle = getShootAngle();
            case FERRY -> targetAngle = getFerryAngle();
        };
    }

    public Rotation2d getTargetAngle() {
        return targetAngle;
    }
    
    public void setHoodState(Hoodstate state) {
        this.hoodstate = state;
    }

    public Hoodstate getHoodState() {
        return hoodstate;
    }

}