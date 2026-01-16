package com.stuypulse.robot.subsystems.hoodedShooter;

import com.ctre.phoenix6.hardware.TalonFX;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Constants;

import edu.wpi.first.math.geometry.Rotation2d;
import com.ctre.phoenix6.controls.PositionVoltage;

public class HoodImpl extends Hood {
    private final TalonFX hoodMotor;

    public HoodImpl() {
        super();
        hoodMotor = new TalonFX(Ports.HDSR.HOOD_MOTOR);
        Motors.Hood.configs.configure(hoodMotor);
    }

    @Override
    public Rotation2d getCurrentAngle() {
        return Rotation2d.fromDegrees(hoodMotor.getPosition().getValueAsDouble());
    }

    @Override 
    public void periodic() {
        hoodMotor.setControl(new PositionVoltage(getTargetAngle().getRotations()));
    }

        
}