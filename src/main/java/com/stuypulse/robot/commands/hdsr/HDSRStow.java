package com.stuypulse.robot.commands.hdsr;

import com.stuypulse.robot.subsystems.hoodedShooter.Hood.Hoodstate;
import com.stuypulse.robot.subsystems.hoodedShooter.Shooter.ShooterState;

public class HDSRStow extends HDSRSetState{
    public HDSRStow() {
        super(Hoodstate.STOW, ShooterState.STOW);
    }
}
