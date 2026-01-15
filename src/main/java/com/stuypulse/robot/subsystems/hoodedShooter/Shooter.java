package com.stuypulse.robot.subsystems.hoodedShooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Shooter extends SubsystemBase {
    
    private static final Shooter instance;
    private ShooterState state;

    static {
        instance = new ShooterImpl();
    }

    public static Shooter getInstance() {
        return instance;
    }

    public enum ShooterState {
        STOW,
        FERRY,
        SHOOT;
    }

    public double getTargetRPM() {
        return switch(getShooterState()) {
            case STOW -> 0;
            case FERRY -> getFerryRPM();
            case SHOOT -> getShootRPM();
        };
    }

    public ShooterState getShooterState() {
        return state;
    }

    public void setShooterState(ShooterState state) {
        this.state = state;
    }

    protected Shooter() {
        state = ShooterState.STOW;
    }

    public abstract double getFerryRPM();
    public abstract double getShootRPM();
    public abstract double getFlywheelRPM();
    public abstract boolean spunUp();


    




    
    
}
