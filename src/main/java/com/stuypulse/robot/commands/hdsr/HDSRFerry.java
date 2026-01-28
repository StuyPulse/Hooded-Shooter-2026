package com.stuypulse.robot.commands.hdsr;

import com.stuypulse.robot.subsystems.hdsr.HDSR.State;

public class HDSRFerry extends HDSRSetState{
    public HDSRFerry() {
        super(State.FERRY);
    }
}