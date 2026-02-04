package com.stuypulse.robot.subsystems.hdsr;
import com.stuypulse.robot.constants.Gains;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.math.Angle;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class HDSRSim extends HDSR{
    private final LinearSystemSim<N2, N1, N2> hood;
    private final FlywheelSim shooter;

    private final AnglePIDController hoodController;
    private final PIDController shooterController;

    public HDSRSim() {
        super();
        
        hood = new LinearSystemSim<N2, N1, N2>(LinearSystemId.identifyPositionSystem(
            3.0,
            0.1
        ));

        shooter = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getKrakenX44(3),
                1.0,
                3.0
            ),
            DCMotor.getKrakenX44(3)
        );
        
        hoodController = new AnglePIDController(
            Gains.Hood.kP,
            Gains.Hood.kI,
            Gains.Hood.kD
        );

        shooterController = new PIDController(
            Gains.Shooter.kP,
            Gains.Shooter.kI,
            Gains.Shooter.kD  
        );
    }

    public Rotation2d getCurrentAngle() {
        return Rotation2d.fromRotations(hood.getOutput(0));
    }

    @Override
    public void periodic() {
        super.periodic();

        hoodController.update(Angle.fromRotation2d(getTargetAngle()), Angle.fromRotation2d(getCurrentAngle()));
        shooterController.update(getTargetRPM(), getCurrentRPS() * 60);
        // SmartDashboard.putNumber("hdsr/Output Voltage", controller);

        hood.setInput(hoodController.getOutput());
        shooter.setInputVoltage(shooterController.getOutput());
    }

    @Override
    public void simulationPeriodic(){
        super.simulationPeriodic();

        hood.update(Settings.DT);
        shooter.update(Settings.DT);      
    }

	@Override
	public double getCurrentRPS() {
        return shooter.getOutput(0);
	}
}
