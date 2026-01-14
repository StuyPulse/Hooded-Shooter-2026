package com.stuypulse.robot.subsystems.hoodedShooter;

import com.ctre.phoenix6.hardware.TalonFX;
import com.stuypulse.robot.constants.Ports;

public class HoodImpl extends Hood {
    private final TalonFX hoodMotor;
    public final HoodImpl hood;

    public HoodImpl() {
        super();
        hoodMotor = new TalonFX(Ports.HDSR.HOOD_MOTOR);
    } 

        
}