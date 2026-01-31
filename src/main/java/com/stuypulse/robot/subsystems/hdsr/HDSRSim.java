package com.stuypulse.robot.subsystems.hdsr;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.math.Angle;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class HDSRSim extends HDSR{
    private final LinearSystemSim<N2, N1, N2> hood;

    private final AnglePIDController hoodController;

    public HDSRSim() {
        super();
        
        hood =  new LinearSystemSim<N2, N1, N2>(LinearSystemId.identifyPositionSystem(
            3.0,
            0.1
        ));
        
        hoodController = new AnglePIDController(
            1.0,
            0.0,
            0.1
        );
    }

    public Rotation2d getCurrentAngle() {
        return Rotation2d.fromRotations(hood.getOutput(0));
    }

    @Override
    public double getFlywheelRPM(){
        return getShootRPM() * 60;
    }

    @Override
    public void periodic() {
        super.periodic();

        hoodController.update(Angle.fromRotation2d(getTargetAngle()), Angle.fromRotation2d(getCurrentAngle()));
    
        hood.setInput(hoodController.getOutput());

    }

    @Override
    public void simulationPeriodic(){
        super.simulationPeriodic();

        hood.update(Settings.DT);        
    }
}
