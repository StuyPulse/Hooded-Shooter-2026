package com.stuypulse.robot.commands;

import com.stuypulse.robot.subsystems.hdsr.HDSR;
import com.stuypulse.robot.subsystems.hdsr.HDSR.State;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.robot.util.ShotCalculator.InterceptSolution;
import com.stuypulse.robot.util.ShotCalculator;
import com.stuypulse.robot.constants.Constants;
import com.stuypulse.robot.constants.Field;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command; 

public class TurretHoodAlignToTarget extends Command{
    private HDSR hdsr;
    private final SwerveDrive swerve;
    private final Odometry odometry;
    // private final Turret turret;

    private Pose3d targetPose;

    // Solve shoot on the fly requires ts params:
    // Pose3d shooterPose,
    // Pose3d targetPose,
    // ChassisSpeeds fieldRelRobotVelocity,
    // ChassisAccelerations fieldRelRobotAcceleration,
    // double targetSpeedRps,
    // int maxIterations,
    // double timeTolerance
    
    public TurretHoodAlignToTarget() {
        hdsr = HDSR.getInstance();
        odometry = Odometry.getInstance();
        swerve = SwerveDrive.getInstance();

        
        addRequirements(hdsr);
    }
     
    @Override
    public void initialize() {
   
    }

    @Override
    public void execute() {
        

        // update targetPose each tick
        if (hdsr.getState() == State.SHOOT) {
            targetPose = Field.hubPose3d;
        }
        else {
            targetPose = Field.hubPose3d; // placeholder
        }



        double RPS = 0;
        if (hdsr.getState() == State.FERRY){
            RPS = Constants.Shooter.FERRY_RPM / 60;
        } else if (hdsr.getState() == State.SHOOT){
            RPS = Constants.Shooter.SHOT_RPM / 60;
        }

        Pose2d currentPose = odometry.getPose();
        SmartDashboard.putNumber("hdsr/rps", RPS);

        InterceptSolution sol = ShotCalculator.solveShootOnTheFly(
            new Pose3d(currentPose), // TODO: add the field relative shooter offset on the robot
            targetPose,
            swerve.getChassisSpeeds(),
            RPS, 
            Constants.Align.MAX_ITERATIONS,
            Constants.Align.TIME_TOLERANCE
        );
        hdsr.setShootAngle(Rotation2d.fromRadians(sol.launchPitchRad())); // TODO: figure out angle range for hood
        
        // this is the required yaw for shooting into the effective hub
        Rotation2d targetTurretAngle = Rotation2d.fromRadians(sol.requiredYaw()).minus(currentPose.getRotation());

        SmartDashboard.putNumber("hdsr/calculated yaw", sol.requiredYaw() *  180 / Math.PI);
        // TODO: set turret angle here                       
    }

    
}