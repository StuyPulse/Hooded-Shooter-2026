package com.stuypulse.robot.subsystems.hdsr;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Constants;
import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class HDSRImpl extends HDSR{
    private final TalonFX hoodMotor;

    private final TalonFX shooterLeader;
    private final TalonFX shooterFollower;

    public HDSRImpl() {
        super();
        
        hoodMotor = new TalonFX(Ports.HDSR.HOOD_MOTOR);
        shooterLeader = new TalonFX(Ports.HDSR.LEADER);
        shooterFollower = new TalonFX(Ports.HDSR.FOLLOWER);

        Motors.Hood.configs.configure(hoodMotor);
        Motors.Hood.configs.configure(shooterLeader);
        Motors.Hood.configs.configure(shooterFollower);
    }

    @Override
    public Rotation2d getCurrentAngle() {
        return Rotation2d.fromDegrees(hoodMotor.getPosition().getValueAsDouble());
    }

    @Override   
    public double getFlywheelRPM() {
        return shooterLeader.getVelocity().getValueAsDouble() * 60;
    }

    @Override 
    public void periodic() {
        super.periodic();

        hoodMotor.setControl(new PositionVoltage(getTargetAngle().getRotations()));

        shooterLeader.setControl(new VelocityVoltage(getTargetRPM() / 60));
        shooterFollower.setControl(new VelocityVoltage(getTargetRPM() / 60));
    }
}