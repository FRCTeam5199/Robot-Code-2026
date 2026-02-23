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
    private Rotation2d lastTurretRotation, turretRotation;
    private double turretVelocity;
    private InterpolatingDoubleTreeMap hoodLookupTable;
    private InterpolatingDoubleTreeMap shooterSpeedLookupTable;
    private InterpolatingDoubleTreeMap timeOfFlightLookupTable;
    private double hoodAngle, lastHoodAngle, hoodVelocity;
    private double shooterSpeed;
    private TurretSubsystem turretSubsystem = TurretSubsystem.getInstance();
    private double turretToTargetDistance = 0;

    private ShotCalculator() {
        hoodLookupTable = new InterpolatingDoubleTreeMap();
        shooterSpeedLookupTable = new InterpolatingDoubleTreeMap();
        timeOfFlightLookupTable = new InterpolatingDoubleTreeMap();

        //key - distance to front center hub
        hoodLookupTable.put(1.499, 0d);
        hoodLookupTable.put(2.501, 0d);
        hoodLookupTable.put(3.507, 0d);

        shooterSpeedLookupTable.put(1.499, 32d);
        shooterSpeedLookupTable.put(2.501, 35d);
        shooterSpeedLookupTable.put(3.507, 39.5);

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
            estimatedPose =
                    estimatedPose.exp(
                            new Twist2d(
                                    robotRelativeVelocity.vxMetersPerSecond * Constants.PHASE_DELAY,
                                    robotRelativeVelocity.vyMetersPerSecond * Constants.PHASE_DELAY,
                                    robotRelativeVelocity.omegaRadiansPerSecond * Constants.PHASE_DELAY));

            //Shifts robot pose to turret pose
            turretPosition = estimatedPose.transformBy(Constants.ROBOT_TO_TURRET);

            // Sets distance values for lookup tables based on ShotMode
            if (RobotContainer.getShotMode() == ShotMode.SHOOTING) {
                this.turretToTargetDistance = Constants.RED_HUB_FRONT_CENTER
                        .getDistance(turretPosition.getTranslation());
            } else if (RobotContainer.getShotMode() == ShotMode.SHUTTLING_LEFT) {
                this.turretToTargetDistance = Constants.SHUTTLE_LEFT_CORNER
                        .getDistance(turretPosition.getTranslation());
            } else if (RobotContainer.getShotMode() == ShotMode.SHUTTLING_RIGHT) {
                this.turretToTargetDistance = Constants.SHUTTLE_RIGHT_CORNER
                        .getDistance(turretPosition.getTranslation());
            }

            //Robot's current velocity
            ChassisSpeeds fieldRelativeVelocity = ChassisSpeeds.fromRobotRelativeSpeeds(
                    RobotContainer.getSpeeds(), estimatedPose.getRotation());
            double robotAngleRadians = estimatedPose.getRotation().getRadians();

            //Turret's current velocity imparted from robot
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

            //Iterate to converge on a final future position
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

            // Sets turret rotations based on ShotMode enums
            if (RobotContainer.getShotMode() == ShotMode.SHOOTING) {
                turretRotation = Constants.RED_HUB_CENTER
                        .minus(futureTurretPosition.getTranslation()).getAngle();
            } else if (RobotContainer.getShotMode() == ShotMode.SHUTTLING_LEFT) {
                turretRotation = Constants.SHUTTLE_LEFT_CORNER
                        .minus(futureTurretPosition.getTranslation()).getAngle();
            } else if (RobotContainer.getShotMode() == ShotMode.SHUTTLING_RIGHT) {
                turretRotation = Constants.SHUTTLE_RIGHT_CORNER
                        .minus(futureTurretPosition.getTranslation()).getAngle();
            }

            hoodAngle = hoodLookupTable.get(lookaheadTurretToTargetDistance);

            if (lastTurretRotation == null) lastTurretRotation = turretRotation;
            if (Double.isNaN(lastHoodAngle)) lastHoodAngle = hoodAngle;

            turretVelocity = turretRotation.minus(lastTurretRotation).getDegrees() / .02;
            hoodVelocity = (hoodAngle - lastHoodAngle) / .02;

            // Sets turret angle and then adjusts it based on robot's rotation in relation to target
            turretAngle = turretRotation.getDegrees();
            turretAngle -= RobotContainer.getPose().getRotation().getDegrees();

            //Wraps to within bounds
            while (turretAngle <= TurretConstants.MIN) turretAngle += 360;
            while (turretAngle >= TurretConstants.MAX) turretAngle -= 360;

            turretVelocity -= (RobotContainer.getSpeeds().omegaRadiansPerSecond / Math.PI * 180d);

            // Based on Future Pose
            shooterSpeed = shooterSpeedLookupTable.get(lookaheadTurretToTargetDistance);
        }

        lastTurretRotation = turretRotation;
        lastHoodAngle = hoodAngle;

        // Sets continuous control for turret, shooter, kicker
        RobotContainer.getTurretControlAuto().setGoal(shotCalculator.getTurretAngle(),
                turretSubsystem.getMotorRotFromDegrees(shotCalculator.getTurretVelocity()));
        RobotContainer.getShooterControlAuto().setGoal(shotCalculator.getShooterSpeed());
        RobotContainer.getKickerControlAuto().setGoal(shotCalculator.getKickerSpeed());
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
        return (shooterSpeed / 2d);
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
}

