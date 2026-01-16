package com.stuypulse.robot.commands.hdsr;

import com.stuypulse.robot.subsystems.hoodedShooter.Hood.Hoodstate;
import com.stuypulse.robot.subsystems.hoodedShooter.Shooter.ShooterState;

public class HDSRShoot extends HDSRSetState {

    // private final Turret turret;
    // swerve

    public HDSRShoot() {
        super(Hoodstate.SHOOT, ShooterState.SHOOT);
    }
}
