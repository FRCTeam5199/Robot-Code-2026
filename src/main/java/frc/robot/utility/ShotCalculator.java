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
    private Pose2d turretPosition, futureTurretPosition;
    private CommandSwerveDrivetrain commandSwerveDrivetrain = RobotContainer.commandSwerveDrivetrain;
    private double turretAngle;
    private double turretAngleWithoutPhaseDelay;
    private Rotation2d lastTurretRotation, turretRotation;
    private double turretVelocity;
    private InterpolatingDoubleTreeMap hoodLookupTable;
    private InterpolatingDoubleTreeMap shooterSpeedLookupTable;
    private InterpolatingDoubleTreeMap timeOfFlightLookupTable;
    private InterpolatingDoubleTreeMap kickerSpeedLookupTable;
    private double hoodAngle, lastHoodAngle, hoodVelocity;
    private double shooterSpeed;
    private double kickerSpeed;
    private TurretSubsystem turretSubsystem = TurretSubsystem.getInstance();
    private double turretToTargetDistance = 0;

    private ShotCalculator() {
        hoodLookupTable = new InterpolatingDoubleTreeMap();
        shooterSpeedLookupTable = new InterpolatingDoubleTreeMap();
        kickerSpeedLookupTable = new InterpolatingDoubleTreeMap();
        timeOfFlightLookupTable = new InterpolatingDoubleTreeMap();

        //key - distance to front center hub
        hoodLookupTable.put(1.499, 0d);
        hoodLookupTable.put(2.501, 0d);
        hoodLookupTable.put(3.507, 0d);

        shooterSpeedLookupTable.put(1.499, 32d);
        shooterSpeedLookupTable.put(2.501, 35d);
        shooterSpeedLookupTable.put(3.507, 39.5);

        kickerSpeedLookupTable.put(1.499, 17d);
        kickerSpeedLookupTable.put(2.501, 18.5);
        kickerSpeedLookupTable.put(3.507, 20.75);


        timeOfFlightLookupTable.put(1.49, (1.14 + 1.10) / 2d);
        timeOfFlightLookupTable.put(2.51, (1.26 + 1.22) / 2d);
        timeOfFlightLookupTable.put(3.52, (1.4 + 1.48) / 2d);

    }

    public static ShotCalculator getInstance() {
        if (shotCalculator == null) shotCalculator = new ShotCalculator();
        return shotCalculator;
    }

    @Override
    public void periodic() {
        if (RobotContainer.getPose() != null) {
            Pose2d estimatedPose = RobotContainer.getPose();
            ChassisSpeeds robotRelativeVelocity = RobotContainer.getSpeeds();

            //phase delay is 0 so this isn't being used right now
//            estimatedPose =
//                    estimatedPose.exp(
//                            new Twist2d(
//                                    robotRelativeVelocity.vxMetersPerSecond * Constants.PHASE_DELAY,
//                                    robotRelativeVelocity.vyMetersPerSecond * Constants.PHASE_DELAY,
//                                    robotRelativeVelocity.omegaRadiansPerSecond * Constants.PHASE_DELAY));

            //Shifts robot pose to turret pose
            turretPosition = estimatedPose.transformBy(Constants.ROBOT_TO_TURRET);
            this.turretToTargetDistance = Constants.RED_HUB_FRONT_CENTER.getDistance(turretPosition.getTranslation());

            //Turrets current velocity
            ChassisSpeeds fieldRelativeVelocity = ChassisSpeeds.fromRobotRelativeSpeeds(
                    RobotContainer.getSpeeds(), estimatedPose.getRotation());
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
            futureTurretPosition = turretPosition;
            double lookaheadTurretToTargetDistance = turretToTargetDistance;

            for (int i = 0; i < 10; i++) {
                timeOfFlight = timeOfFlightLookupTable.get(lookaheadTurretToTargetDistance);
                double offsetX = turretVelocityX * timeOfFlight;
                double offsetY = turretVelocityY * timeOfFlight;
                futureTurretPosition =
                        new Pose2d(
                                turretPosition.getTranslation().plus(new Translation2d(offsetX, offsetY)),
                                turretPosition.getRotation());
                lookaheadTurretToTargetDistance = Constants.RED_HUB_FRONT_CENTER.getDistance(futureTurretPosition.getTranslation());
            }

            turretRotation = Constants.RED_HUB_CENTER
                    .minus(futureTurretPosition.getTranslation()).getAngle();

            hoodAngle = hoodLookupTable.get(lookaheadTurretToTargetDistance);

            if (lastTurretRotation == null) lastTurretRotation = turretRotation;
            if (Double.isNaN(lastHoodAngle)) lastHoodAngle = hoodAngle;

            turretVelocity = turretRotation.minus(lastTurretRotation).getDegrees() / .02;
            hoodVelocity = (hoodAngle - lastHoodAngle) / .02;

            turretAngle = turretRotation.getDegrees();
            turretAngle -= RobotContainer.getPose().getRotation().getDegrees();

            //Wraps to within bounds
            while (turretAngle <= TurretConstants.TURRET_MIN) turretAngle += 360;
            while (turretAngle >= TurretConstants.TURRET_MAX) turretAngle -= 360;

            turretVelocity -= (RobotContainer.getSpeeds().omegaRadiansPerSecond / Math.PI * 180d);

            shooterSpeed = shooterSpeedLookupTable.get(lookaheadTurretToTargetDistance);
            kickerSpeed = kickerSpeedLookupTable.get(lookaheadTurretToTargetDistance);
        }

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

    public double getKickerSpeed() {
        return kickerSpeed;
    }


    public double getTurretAngle() {
        return turretAngle;
    }

    public double getTurretVelocity() {
        return turretVelocity;
    }

    public double getTurretToTargetDistance() {
        return turretToTargetDistance;
    }

    public Pose2d getTurretPosition() {
        return turretPosition;
    }

    public Pose2d getFutureTurretPosition() {
        return futureTurretPosition;
    }

    public double getTurretAngleWithoutPhaseDelay() {
        return turretAngleWithoutPhaseDelay;
    }
}

