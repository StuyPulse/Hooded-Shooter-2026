package com.stuypulse.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;

public class Constants {
    public interface Hood {
        Rotation2d MIN_ANGLE = Rotation2d.fromDegrees(0);
        Rotation2d MAX_ANGLE = Rotation2d.fromDegrees(0);
        Rotation2d SHOOT_ANGLE = Rotation2d.fromDegrees(0);
    }
    public interface Shooter {
        double SHOT_RPM = 0;
        double FERRY_RPM = 0;
    }
}