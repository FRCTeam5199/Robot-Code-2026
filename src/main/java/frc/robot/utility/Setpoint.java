package frc.robot.utility;

import frc.robot.constants.HoodConstants;

public enum Setpoint {
    TOWER(15, 23.2),
    SHUTTLE(HoodConstants.MAX, 38);

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
