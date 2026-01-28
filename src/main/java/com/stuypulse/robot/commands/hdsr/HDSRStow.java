package com.stuypulse.robot.commands.hdsr;

import com.stuypulse.robot.subsystems.hdsr.HDSR.State;

public class HDSRStow extends HDSRSetState{
    public HDSRStow() {
        super(State.STOW);
    }
}
