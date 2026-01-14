package com.stuypulse.robot.subsystems.hoodedShooter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Hood extends SubsystemBase {
    private static final Hood instance;

    static {
        instance = new HoodImpl();
    }

    public static Hood getInstance() {
        return instance;
    }

    private double currentAngle;
    private double targetAngle;

    protected Hood() {

    }

    public abstract double getCurrentAngle();
    public abstract double getTargetAngle();
    public abstract double setTargetAngle(double angle);
    
    public void zeroAngle() {
        setTargetAngle(0);
    }
}