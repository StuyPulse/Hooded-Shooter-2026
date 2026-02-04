package com.stuypulse.robot.commands;

import com.stuypulse.robot.subsystems.hdsr.HDSR;
import com.stuypulse.robot.subsystems.hdsr.HDSR.State;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.robot.util.ShotCalculator.AlignAngleSolution;
import com.stuypulse.robot.util.ShotCalculator;
import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Constants;
import com.stuypulse.robot.constants.Field;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command; 

public class TurretHoodAlignToTarget extends Command{
    private HDSR hdsr;
    private final SwerveDrive swerve;
    private final Odometry odometry;
    // private final Turret turret;

    private final Field2d field;

    private Pose3d targetPose;

    private FieldObject2d targetPose2d;
    private FieldObject2d virtualHubPose2d;

    private ChassisSpeeds prevfieldRelRobotSpeeds;
    private ChassisSpeeds fieldRelRobotSpeeds;

    
    public TurretHoodAlignToTarget() {
        hdsr = HDSR.getInstance();
        odometry = Odometry.getInstance();
        swerve = SwerveDrive.getInstance();

        field = Field.FIELD2D;
        virtualHubPose2d = field.getObject("virtualHubPose");
        targetPose2d = field.getObject("targetPose");

        prevfieldRelRobotSpeeds = new ChassisSpeeds();
        fieldRelRobotSpeeds = new ChassisSpeeds();

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

        targetPose = Robot.isBlue() ? targetPose : Field.transformToOppositeAlliance(targetPose);

        Pose2d currentPose = Robot.isBlue() ? odometry.getPose() : Field.transformToOppositeAlliance(odometry.getPose());

        prevfieldRelRobotSpeeds = fieldRelRobotSpeeds;
        fieldRelRobotSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(swerve.getChassisSpeeds(), currentPose.getRotation());
        
        AlignAngleSolution sol = ShotCalculator.solveShootOnTheFly(
            new Pose3d(currentPose), // TODO: add the field relative shooter offset on the robot
            targetPose,
            prevfieldRelRobotSpeeds,
            fieldRelRobotSpeeds,
            hdsr.getCurrentRPS(), 
            Constants.Align.MAX_ITERATIONS,
            Constants.Align.TIME_TOLERANCE
        );

        hdsr.setTargetAngle(sol.launchPitchAngle());
        
        // this is the required yaw for shooting into the effective hub
        Rotation2d targetTurretAngle = sol.requiredYaw().minus(currentPose.getRotation()) ;

        targetPose2d.setPose(targetPose.toPose2d());
        virtualHubPose2d.setPose(sol.estimateTargetPose().toPose2d());
  
        SmartDashboard.putNumber("hdsr/calculated yaw", targetTurretAngle.getDegrees());
        // TODO: set turret angle here    
                         
    }

    
}