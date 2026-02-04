package frc.robot.utility;

public enum Setpoint {
    HUB(0, 0, 30),  //01
    TOWER(0, 7, 35),  //7
    LEFT_CORNER(37, 9, 42.5),  //9
    OUTPOST(-37, 9, 40.5);      //hood: 9, shooter:43

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
}
