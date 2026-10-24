package frc.robot.utility;

public enum Setpoint {
    TOWER(0, 47), //intake against hub
    TRENCH(13, 60.5),
    SHUTTLE(20, 68.5);

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
