package com.stuypulse.robot.commands.hdsr;

import com.stuypulse.robot.subsystems.hoodedShooter.Hood;
import com.stuypulse.robot.subsystems.hoodedShooter.Hood.Hoodstate;
import com.stuypulse.robot.subsystems.hoodedShooter.Shooter.ShooterState;
import com.stuypulse.robot.subsystems.hoodedShooter.Shooter;

import edu.wpi.first.wpilibj2.command.Command;


public class HDSRSetState extends Command{
    private final Shooter shooter;
    private final Hood hood;
    private final Hoodstate hoodstate;
    private final ShooterState shooterState;
    
    public HDSRSetState(Hoodstate hState, ShooterState sState){ 
        hood = Hood.getInstance();
        shooter = Shooter.getInstance();
        hoodstate = hState;
        shooterState = sState;
        addRequirements(hood, shooter);
    }    

    @Override
    public void initialize(){
        hood.setHoodState(hoodstate);
        shooter.setShooterState(shooterState);
    }
}
