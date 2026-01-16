package com.stuypulse.robot.commands;

import com.stuypulse.robot.subsystems.hoodedShooter.Hood;

import edu.wpi.first.wpilibj2.command.Command; 

public class TurretHoodAlignToTarget extends Command{
    private final Hood hood;
    // swerve turret odometry

    // Solve shoot on the fly requires ts params:
    // Pose3d shooterPose,
    // Pose3d targetPose,
    // ChassisSpeeds fieldRelRobotVelocity,
    // ChassisAccelerations fieldRelRobotAcceleration,
    // double targetSpeedRps,
    // int maxIterations,
    // double timeTolerance
    
    public TurretHoodAlignToTarget(){
        hood = Hood.getInstance();
        addRequirements(hood);
    }
     
}
