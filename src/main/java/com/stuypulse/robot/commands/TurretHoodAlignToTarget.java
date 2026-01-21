package com.stuypulse.robot.commands;

import com.stuypulse.robot.subsystems.hoodedShooter.Hood;
import com.stuypulse.robot.subsystems.hoodedShooter.Shooter;
import com.stuypulse.robot.subsystems.hoodedShooter.Shooter.ShooterState;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.robot.util.ShotCalculator.InterceptSolution;
import com.stuypulse.robot.util.ShotCalculator;
import com.stuypulse.robot.constants.Constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command; 

public class TurretHoodAlignToTarget extends Command{
    private final Hood hood;
    private final Shooter shooter;
    private final SwerveDrive swerve;
    private final Odometry odometry;
    // private final Turret turret;

    private final Pose3d targetPose;

    // Solve shoot on the fly requires ts params:
    // Pose3d shooterPose,
    // Pose3d targetPose,
    // ChassisSpeeds fieldRelRobotVelocity,
    // ChassisAccelerations fieldRelRobotAcceleration,
    // double targetSpeedRps,
    // int maxIterations,
    // double timeTolerance
    
    public TurretHoodAlignToTarget(Pose3d targetPose){
        hood = Hood.getInstance();
        shooter = Shooter.getInstance();
        odometry = Odometry.getInstance();
        swerve = SwerveDrive.getInstance();

        this.targetPose = targetPose;
        addRequirements(hood);
    }
     
    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double RPS = 0;
        if (shooter.getShooterState() == ShooterState.FERRY){
            RPS = Constants.Shooter.FERRY_RPM / 60;
        } else if (shooter.getShooterState() == ShooterState.SHOOT){
            RPS = Constants.Shooter.SHOT_RPM / 60;
        }

        Pose2d currentPose = odometry.getPose();

        InterceptSolution sol = ShotCalculator.solveShootOnTheFly(
            new Pose3d(currentPose), // TODO: add the field relative shooter offset on the robot
            targetPose,
            swerve.getChassisSpeeds(),
            RPS, 
            Constants.Align.MAX_ITERATIONS,
            Constants.Align.TIME_TOLERANCE
        );
        hood.setShootAngle(Rotation2d.fromRadians(sol.launchPitchRad())); // TODO: figure out angle range for hood
        
        // this is the required yaw for shooting into the effective hub
        Rotation2d targetTurretAngle = Rotation2d.fromRadians(sol.requiredYaw()).plus(currentPose.getRotation());

        // TODO: set turret angle here                       
    } 
}