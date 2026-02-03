package frc.robot.utility;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class ShotCalculator extends SubsystemBase {
    private static ShotCalculator shotCalculator;
    private Pose2d turretPosition;
    private CommandSwerveDrivetrain commandSwerveDrivetrain = RobotContainer.commandSwerveDrivetrain;
    private double turretAngle;
    private InterpolatingDoubleTreeMap hoodLookupTable;
    private InterpolatingDoubleTreeMap shooterSpeedLookupTable;
    private double hoodAngle;
    private double shooterSpeed;

    private ShotCalculator() {
        hoodLookupTable = new InterpolatingDoubleTreeMap();
        shooterSpeedLookupTable = new InterpolatingDoubleTreeMap();

        //key - distance to front center hub
        hoodLookupTable.put(1.0, 1.0);
        shooterSpeedLookupTable.put(1.0, 1.0);
    }

    public static ShotCalculator getInstance() {
        if (shotCalculator == null) shotCalculator = new ShotCalculator();
        return shotCalculator;
    }

    @Override
    public void periodic() {
        if (commandSwerveDrivetrain.getPose() != null) {
            turretPosition = commandSwerveDrivetrain.getPose().transformBy(Constants.ROBOT_TO_TURRET);
            turretAngle = Constants.RED_HUB_CENTER.minus(turretPosition.getTranslation()).getAngle().getDegrees();
            if (turretAngle < -180) turretAngle += 360;

            turretAngle -= commandSwerveDrivetrain.getPose().getRotation().getDegrees();
            if (turretAngle <= -180) turretAngle += 360;
            if (turretAngle >= 180) turretAngle -= 360;
            // System.out.println("Turret Angle: " + turretAngle);
        }

        hoodAngle = hoodLookupTable.get(Constants.RED_HUB_FRONT_CENTER.getDistance(turretPosition.getTranslation()));
        shooterSpeed = shooterSpeedLookupTable.get(Constants.RED_HUB_FRONT_CENTER.getDistance(turretPosition.getTranslation()));
    }
    public double getHoodAngle() {
        return hoodAngle;
    }

    public double getShooterSpeed() {
        return shooterSpeed;
    }

    public double getTurretAngle() {
        return turretAngle;
    }
}
