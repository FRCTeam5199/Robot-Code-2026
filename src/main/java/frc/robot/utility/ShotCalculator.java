package frc.robot.utility;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
import frc.robot.constants.TurretConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.TurretSubsystem;

public class ShotCalculator extends SubsystemBase {
    private static ShotCalculator shotCalculator;
    //    private final LinearFilter turretAngleFilter =
//            LinearFilter.movingAverage((int) (0.1 / 0.02));
    private Pose2d turretPosition;
    private CommandSwerveDrivetrain commandSwerveDrivetrain = RobotContainer.commandSwerveDrivetrain;
    private double turretAngle;
    private Rotation2d lastTurretRotation, turretRotation;
    private double turretVelocity;
    private InterpolatingDoubleTreeMap hoodLookupTable;
    private InterpolatingDoubleTreeMap shooterSpeedLookupTable;
    private InterpolatingDoubleTreeMap timeOfFlightLookupTable;
    private double hoodAngle, lastHoodAngle, hoodVelocity;
    private double shooterSpeed;
    private TurretSubsystem turretSubsystem = TurretSubsystem.getInstance();

    private ShotCalculator() {
        hoodLookupTable = new InterpolatingDoubleTreeMap();
        shooterSpeedLookupTable = new InterpolatingDoubleTreeMap();
        timeOfFlightLookupTable = new InterpolatingDoubleTreeMap();

        //key - distance to front center hub
        hoodLookupTable.put(1.0, 1.0);
        shooterSpeedLookupTable.put(1.0, 1.0);
        timeOfFlightLookupTable.put(1d, 1d);
    }

    public static ShotCalculator getInstance() {
        if (shotCalculator == null) shotCalculator = new ShotCalculator();
        return shotCalculator;
    }

    @Override
    public void periodic() {
        if (commandSwerveDrivetrain.getPose() != null) {
            Pose2d estimatedPose = commandSwerveDrivetrain.getPose();
//            ChassisSpeeds robotRelativeVelocity = commandSwerveDrivetrain.getState().Speeds;
//            estimatedPose =
//                    estimatedPose.exp(
//                            new Twist2d(
//                                    robotRelativeVelocity.vxMetersPerSecond * TurretConstants.PHASE_DELAY,
//                                    robotRelativeVelocity.vyMetersPerSecond * TurretConstants.PHASE_DELAY,
//                                    robotRelativeVelocity.omegaRadiansPerSecond * TurretConstants.PHASE_DELAY));

            //Turrets current position
            turretPosition = estimatedPose.transformBy(Constants.ROBOT_TO_TURRET);
            double turretToTargetDistance = Constants.RED_HUB_FRONT_CENTER.getDistance(turretPosition.getTranslation());

            //Turrets current velocity
            ChassisSpeeds fieldRelativeVelocity = ChassisSpeeds.fromRobotRelativeSpeeds(
                    commandSwerveDrivetrain.getState().Speeds, estimatedPose.getRotation());
            double robotAngleRadians = estimatedPose.getRotation().getRadians();
            double turretVelocityX =
                    fieldRelativeVelocity.vxMetersPerSecond
                            + fieldRelativeVelocity.omegaRadiansPerSecond
                            * (Constants.ROBOT_TO_TURRET.getY() * Math.cos(robotAngleRadians)
                            - Constants.ROBOT_TO_TURRET.getX() * Math.sin(robotAngleRadians));
            double turretVelocityY =
                    fieldRelativeVelocity.vyMetersPerSecond
                            + fieldRelativeVelocity.omegaRadiansPerSecond
                            * (Constants.ROBOT_TO_TURRET.getX() * Math.cos(robotAngleRadians)
                            - Constants.ROBOT_TO_TURRET.getY() * Math.sin(robotAngleRadians));

            //Account for velocity
            double timeOfFlight;
            Pose2d lookaheadTurretPose = turretPosition;
            double lookaheadTurretToTargetDistance = turretToTargetDistance;

            //Commented out so we can test stationary
//            for (int i = 0; i < 20; i++) {
//                timeOfFlight = timeOfFlightLookupTable.get(lookaheadTurretToTargetDistance);
//                double offsetX = turretVelocityX * timeOfFlight;
//                double offsetY = turretVelocityY * timeOfFlight;
//                lookaheadTurretPose =
//                        new Pose2d(
//                                turretPosition.getTranslation().plus(new Translation2d(offsetX, offsetY)),
//                                turretPosition.getRotation());
//                lookaheadTurretToTargetDistance = Constants.RED_HUB_FRONT_CENTER.getDistance(lookaheadTurretPose.getTranslation());
//            }

            turretRotation = Constants.RED_HUB_CENTER
                    .minus(lookaheadTurretPose.getTranslation()).getAngle();
            hoodAngle = hoodLookupTable.get(lookaheadTurretToTargetDistance);

            if (lastTurretRotation == null) lastTurretRotation = turretRotation;
            if (Double.isNaN(lastHoodAngle)) lastHoodAngle = hoodAngle;


            turretVelocity = turretRotation.minus(lastTurretRotation).getDegrees() / .02;
            hoodVelocity = (hoodAngle - lastHoodAngle) / .02;

            turretAngle = turretRotation.getDegrees();
            turretAngle -= commandSwerveDrivetrain.getPose().getRotation().getDegrees();
            while (turretAngle <= -180) turretAngle += 360;
            while (turretAngle >= 180) turretAngle -= 360;

            turretVelocity -= commandSwerveDrivetrain.getState().Speeds.omegaRadiansPerSecond / Math.PI * 180d;

            shooterSpeed = shooterSpeedLookupTable.get(lookaheadTurretToTargetDistance);
        }

//        hoodAngle = hoodLookupTable.get(Constants.RED_HUB_FRONT_CENTER.getDistance(turretPosition.getTranslation()));
//        shooterSpeed = shooterSpeedLookupTable.get(Constants.RED_HUB_FRONT_CENTER.getDistance(turretPosition.getTranslation()));

        lastTurretRotation = turretRotation;
        lastHoodAngle = hoodAngle;
    }

    public double getHoodAngle() {
        return hoodAngle;
    }

    public double getHoodVelocity() {
        return hoodVelocity;
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
