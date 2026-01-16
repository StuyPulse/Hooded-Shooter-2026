package com.stuypulse.robot.constants;

import com.pathplanner.lib.config.PIDConstants;
import com.stuypulse.stuylib.network.SmartNumber;

public interface Gains {

    public interface Swerve {

        public interface Alignment {
            PIDConstants XY = new PIDConstants(2.0, 0, 0.02);
            PIDConstants THETA = new PIDConstants(3, 0, 0.1);
        }

        public interface Turn {
            double kP = 2.0;
            double kI = 0.0;
            double kD = 0.05;
        }

        public interface Drive {
            double kP = 0.5;
            double kI = 0.0;
            double kD = 0.05;

            double kS = 0.26722;
            double kV = 2.2119;
            double kA = 0.36249;
        }
    }
}
