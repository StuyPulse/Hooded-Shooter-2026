package com.stuypulse.robot.subsystems.hdsr;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import edu.wpi.first.math.geometry.Rotation2d;

public class HDSRImpl extends HDSR{
    private final TalonFX hoodMotor;

    private final TalonFX shooterLeader;
    private final TalonFX shooterFollower1;
    private final TalonFX shooterFollower2;

    public HDSRImpl() {
        super();
        
        hoodMotor = new TalonFX(Ports.HDSR.HOOD_MOTOR);
        shooterLeader = new TalonFX(Ports.HDSR.LEADER);
        shooterFollower1 = new TalonFX(Ports.HDSR.FOLLOWER1);
        shooterFollower2 = new TalonFX(Ports.HDSR.FOLLOWER2);


        Motors.Hood.configs.configure(hoodMotor);
        Motors.Hood.configs.configure(shooterLeader);
        Motors.Hood.configs.configure(shooterFollower1);
        Motors.Hood.configs.configure(shooterFollower2);
    }

    @Override
    public double getCurrentRPS(){
        return shooterLeader.getVelocity().getValueAsDouble();
    }

    @Override
    public Rotation2d getCurrentAngle() {
        return Rotation2d.fromDegrees(hoodMotor.getPosition().getValueAsDouble());
    }

    @Override 
    public void periodic() {
        super.periodic();

        hoodMotor.setControl(new PositionVoltage(getTargetAngle().getRotations()));

        shooterLeader.setControl(new VelocityVoltage(getTargetRPM() / 60));
        shooterFollower1.setControl(new Follower(shooterLeader.getDeviceID(), MotorAlignmentValue.Aligned));
        shooterFollower2.setControl(new Follower(shooterLeader.getDeviceID(), MotorAlignmentValue.Aligned));

    }
}