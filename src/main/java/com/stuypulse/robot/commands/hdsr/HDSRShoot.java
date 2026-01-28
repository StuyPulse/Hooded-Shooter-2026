package com.stuypulse.robot.commands.hdsr;


import com.stuypulse.robot.subsystems.hdsr.HDSR.State;

public class HDSRShoot extends HDSRSetState {

    // private final Turret turret;
    // swerve

    public HDSRShoot() {
        super(State.SHOOT);
    }
}
