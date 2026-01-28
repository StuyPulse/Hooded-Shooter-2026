// BORROWED CODE FROM 868!!! some stuffs added/edited like yaw calcumulations. gracias

package com.stuypulse.robot.util;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public final class ShotCalculator {
    public static final double g = 9.81; // gravity is not a number

    public record ShotSolution(
        double launchPitchRad,
        double launchSpeed,
        double flightTimeSeconds) {
    }

    private ShotCalculator() {
    }

    public static ShotSolution solveBallisticWithSpeed(
            Pose3d shooterPose,
            Pose3d targetPose,
            double launchSpeed) {

        Translation3d s = shooterPose.getTranslation();
        Translation3d t = targetPose.getTranslation();

        double dx = t.getX() - s.getX();
        double dy = t.getY() - s.getY();
        double dz = t.getZ() - s.getZ();

        double d = Math.hypot(dx, dy);
        if (d < 1e-9) {
            throw new IllegalArgumentException("Horizontal distance too small");
        }

        double v2 = launchSpeed * launchSpeed;

        double discriminant = v2 * v2 - g * (g * d * d + 2.0 * dz * v2);
        if (discriminant < 0) {
            return new ShotSolution(0, 0, 0);
        }

        // LOW-ARC solution (use + for high arc)
        double tanTheta = (v2 - Math.sqrt(discriminant)) / (g * d);

        double launchPitch = Math.atan(tanTheta);

        double vHoriz = launchSpeed * Math.cos(launchPitch);
        double time = d / vHoriz;

        return new ShotSolution(launchPitch, launchSpeed, time);
    }


    public record InterceptSolution(
        Pose3d effectiveTargetPose,
        double launchPitchRad,
        double launchSpeed,
        double flightTime,
        double requiredYaw) {
    }

    public static InterceptSolution solveShootOnTheFly(
        Pose3d shooterPose,
        Pose3d targetPose,
        ChassisSpeeds fieldRelRobotVelocity,
        double targetSpeedRps,
        int maxIterations,
        double timeTolerance) {

        ShotSolution sol = solveBallisticWithSpeed(
            shooterPose,
            targetPose,
            targetSpeedRps);

        
        double t = sol.flightTimeSeconds();
        
        Pose3d effectiveTarget = targetPose;
        Translation3d s = shooterPose.getTranslation();
            
        for (int i = 0; i < maxIterations; i++) {

            double dx = fieldRelRobotVelocity.vxMetersPerSecond * t;
            // + 0.5 * fieldRelRobotAcceleration.axMetersPerSecondSquared * t * t;

            double dy = fieldRelRobotVelocity.vyMetersPerSecond * t;
            // + 0.5 * fieldRelRobotAcceleration.ayMetersPerSecondSquared * t * t;

            effectiveTarget = new Pose3d(
                targetPose.getX() - dx,
                targetPose.getY() - dy,
                targetPose.getZ(),
                targetPose.getRotation());

            ShotSolution newSol = solveBallisticWithSpeed(
                shooterPose,
                effectiveTarget,
                targetSpeedRps);

            sol = newSol;
            if (Math.abs(newSol.flightTimeSeconds() - t) < timeTolerance) {
                break;
            }

            t = newSol.flightTimeSeconds();
        }

        
        Translation3d et = effectiveTarget.getTranslation();

        // all the poses we pass in are field relative, 
        // so calculated yaw (turret angle) should be field relative as well... right

        double yaw = Math.atan2( //atan2(dy, dx)
            et.getY() - s.getY(),
            et.getX() - s.getX() 
        ); 

        return new InterceptSolution(
            effectiveTarget,
            sol.launchPitchRad(),
            sol.launchSpeed(),
            sol.flightTimeSeconds(),
            yaw);
    }
}