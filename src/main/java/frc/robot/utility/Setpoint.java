package frc.robot.utility;

import frc.robot.constants.HoodConstants;

public enum Setpoint {
    TOWER(13, 22.9),
    SHUTTLELONG(HoodConstants.MAX, 38),
    SHUTTLESHORT(18, 25),
    CHRISLAN(17, 30);

    private final double  hoodAngle, shooterSpeed;

    Setpoint(double hoodAngle, double shooterSpeed) {
        this.hoodAngle = hoodAngle;
        this.shooterSpeed = shooterSpeed;
    }

    

    public double getHoodAngle() {
        return hoodAngle;
    }

    public double getShooterSpeed() {
        return shooterSpeed;
    }

}
