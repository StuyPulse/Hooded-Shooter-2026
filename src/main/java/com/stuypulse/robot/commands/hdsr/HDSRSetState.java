package com.stuypulse.robot.commands.hdsr;

import com.stuypulse.robot.subsystems.hdsr.HDSR;
import com.stuypulse.robot.subsystems.hdsr.HDSR.State;

import edu.wpi.first.wpilibj2.command.InstantCommand;


public class HDSRSetState extends InstantCommand{
    private final HDSR hdsr;
    private final State state; 
    
    public HDSRSetState(State state){ 
        hdsr = HDSR.getInstance();
        this.state = state;
        
        addRequirements(hdsr);
    }    

    @Override
    public void initialize(){
        hdsr.setState(state);
    }
}
