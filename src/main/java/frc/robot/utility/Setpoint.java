package frc.robot.utility;

import frc.robot.constants.HoodConstants;

public enum Setpoint {
    TOWER(13, 65),
    TRENCH(12, 60),
    SHUTTLE(HoodConstants.MAX, 100);

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
