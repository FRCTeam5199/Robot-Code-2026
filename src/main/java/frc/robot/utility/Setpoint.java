package frc.robot.utility;

public enum Setpoint {
    HUB(0, 0, 28), //28 //intake against hub
    TOWER(0, 5, 32),
    LEFT_CORNER(-45, 10, 39),
    OUTPOST(36, 10, 38);

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
        return (shooterSpeed / 2.5);
    }
}
