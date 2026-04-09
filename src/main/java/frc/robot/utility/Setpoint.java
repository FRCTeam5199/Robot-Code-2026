package frc.robot.utility;

import frc.robot.constants.KickerConstants;

public enum Setpoint {
    HUB(0, 0, 47), //28 //intake against hub
    TOWER(-10, 13, 60.5),
    LEFT_CORNER(-40, 20, 68.5),
    OUTPOST(38, 20, 67);

    private final double turretAngle, hoodAngle, shooterSpeed;

    Setpoint(double turretAngle, double hoodAngle, double shooterSpeed) {
        this.turretAngle = turretAngle;
        this.hoodAngle = hoodAngle;
        this.shooterSpeed = shooterSpeed;
    }

    public double getTurretAngle() {
        return turretAngle;
    }

    public double getHoodAngle() {
        return hoodAngle;
    }

    public double getShooterSpeed() {
        return shooterSpeed;
    }

    public double getKickerSpeed() {
        return KickerConstants.INDEXING_SPEED;
    }
}
