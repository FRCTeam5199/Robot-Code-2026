package frc.robot.utility;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
import frc.robot.constants.TurretConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.TurretSubsystem;

public class ShotCalculator extends SubsystemBase {
    private static ShotCalculator shotCalculator;
    private final LinearFilter turretAngleFilter =
            LinearFilter.movingAverage((int) (0.1 / 0.02));
    private Pose2d turretPosition;
    private CommandSwerveDrivetrain commandSwerveDrivetrain = RobotContainer.commandSwerveDrivetrain;
    private Rotation2d turretRotation;
    private double turretAngle;
    private Rotation2d lastTurretRotation;
    private double turretVelocity;
    private InterpolatingDoubleTreeMap hoodLookupTable;
    private InterpolatingDoubleTreeMap shooterSpeedLookupTable;
    private double hoodAngle;
    private double shooterSpeed;
    private TurretSubsystem turretSubsystem = TurretSubsystem.getInstance();

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
            Pose2d estimatedPose = commandSwerveDrivetrain.getPose();
            ChassisSpeeds robotRelativeVelocity = commandSwerveDrivetrain.getState().Speeds;
            estimatedPose =
                    estimatedPose.exp(
                            new Twist2d(
                                    robotRelativeVelocity.vxMetersPerSecond * TurretConstants.PHASE_DELAY,
                                    robotRelativeVelocity.vyMetersPerSecond * TurretConstants.PHASE_DELAY,
                                    robotRelativeVelocity.omegaRadiansPerSecond * TurretConstants.PHASE_DELAY));
//            double turretToTargetDistance = Constants.RED_HUB_CENTER.getDistance(turretPosition.getTranslation());
            turretPosition = estimatedPose.transformBy(Constants.ROBOT_TO_TURRET);

            turretRotation = Constants.RED_HUB_CENTER
                    .minus(turretPosition.getTranslation()).getAngle();

            if (lastTurretRotation == null) lastTurretRotation = turretRotation;
            turretVelocity = turretRotation.minus(lastTurretRotation).getDegrees() / .02;

            turretAngle = turretRotation.getDegrees();
            turretAngle -= commandSwerveDrivetrain.getPose().getRotation().getDegrees();
            while (turretAngle <= -180) turretAngle += 360;
            while (turretAngle >= 180) turretAngle -= 360;

            turretVelocity -= commandSwerveDrivetrain.getState().Speeds.omegaRadiansPerSecond / Math.PI * 180d;
        }

//        hoodAngle = hoodLookupTable.get(Constants.RED_HUB_FRONT_CENTER.getDistance(turretPosition.getTranslation()));
//        shooterSpeed = shooterSpeedLookupTable.get(Constants.RED_HUB_FRONT_CENTER.getDistance(turretPosition.getTranslation()));

        lastTurretRotation = turretRotation;
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

    public double getTurretVelocity() {
        return turretVelocity;
    }
}
