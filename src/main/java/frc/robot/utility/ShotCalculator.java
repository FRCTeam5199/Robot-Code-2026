package frc.robot.utility;

import edu.wpi.first.math.Pair;
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
import frc.robot.constants.KickerConstants;
import frc.robot.constants.TurretConstants;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.utility.AllianceFlipper;

import javax.security.auth.kerberos.KerberosCredMessage;
import java.util.concurrent.BlockingDeque;

public class ShotCalculator extends SubsystemBase {
    private static ShotCalculator shotCalculator;
    //    private final LinearFilter turretAngleFilter =
//            LinearFilter.movingAverage((int) (0.1 / 0.02));
    private Pose2d futureTurretPosition, futureTurretPositionPhaseDelayed;
    private double turretAngle, turretAnglePhaseDelayed;
    private Rotation2d lastTurretRotation, lastTurretRotationPhaseDelayed, turretRotation, turretRotationPhaseDelayed;
    private double turretVelocity, turretVelocityPhaseDelayed;
    private InterpolatingDoubleTreeMap hoodLookupTable;
    private InterpolatingDoubleTreeMap shooterSpeedLookupTable;
    private InterpolatingDoubleTreeMap timeOfFlightLookupTable;
    private InterpolatingDoubleTreeMap shuttleHoodLookupTable;
    private InterpolatingDoubleTreeMap shuttleShooterSpeedLookupTable;
    private InterpolatingDoubleTreeMap shuttleTimeOfFlightLookupTable;
    private double hoodAnglePhaseDelayed, lastHoodAnglePhaseDelayed, hoodAngle, lastHoodAngle, hoodVelocityPhaseDelayed, hoodVelocity;
    private double shooterSpeed, shooterSpeedPhaseDelayed;
    private TurretSubsystem turretSubsystem = TurretSubsystem.getInstance();
    private double futureTurretToTargetDistance, futureTurretToTargetDistancePhaseDelayed = 0;

    private ShotCalculator() {
        hoodLookupTable = new InterpolatingDoubleTreeMap();
        shooterSpeedLookupTable = new InterpolatingDoubleTreeMap();
        timeOfFlightLookupTable = new InterpolatingDoubleTreeMap();

        shuttleHoodLookupTable = new InterpolatingDoubleTreeMap();
        shuttleShooterSpeedLookupTable = new InterpolatingDoubleTreeMap();
        shuttleTimeOfFlightLookupTable = new InterpolatingDoubleTreeMap();

//        hoodLookupTable.put(1.013 + Constants.HUB_RADIUS, 0d);
        hoodLookupTable.put(1.968 + Constants.HUB_RADIUS, 6d);
        hoodLookupTable.put(3.049 + Constants.HUB_RADIUS, 12d);
        hoodLookupTable.put(4.0513 + Constants.HUB_RADIUS, 16d);
        hoodLookupTable.put(5d + Constants.HUB_RADIUS, 24d);

        shooterSpeedLookupTable.put(1.968 + Constants.HUB_RADIUS, 55d);
        shooterSpeedLookupTable.put(3.049 + Constants.HUB_RADIUS, 56d);
        shooterSpeedLookupTable.put(4.0513 + Constants.HUB_RADIUS, 62d);
        shooterSpeedLookupTable.put(5d + Constants.HUB_RADIUS, 68d);
        //Motor Rotations = degrees / 360 / .01255707762557077625570776255708
        //Degrees = motorRot * 360 * .01255707762557077625570776255708

//        timeOfFlightLookupTable.put(1.013 + Constants.HUB_RADIUS, .88 - .15 - .02);
        timeOfFlightLookupTable.put(1.968 + Constants.HUB_RADIUS, 1.05);
        timeOfFlightLookupTable.put(3.049 + Constants.HUB_RADIUS, 0.99);
        timeOfFlightLookupTable.put(4.0513 + Constants.HUB_RADIUS, 0.99);
        timeOfFlightLookupTable.put(5d + Constants.HUB_RADIUS, 1.1);

        //----------------
        shuttleHoodLookupTable.put(7d, 0d);
//        shuttleHoodLookupTable.put(8.5d, 14d);
//        shuttleHoodLookupTable.put(10d, 18d);
//        shuttleHoodLookupTable.put(14d, 24d);

        shuttleShooterSpeedLookupTable.put(5.5, 30d - 3d);
        shuttleShooterSpeedLookupTable.put(7d, 37d - 3d);
        shuttleShooterSpeedLookupTable.put(8.5d, 39d - 3d);
        shuttleShooterSpeedLookupTable.put(14d, 60d - 3d);

        shuttleTimeOfFlightLookupTable.put(5.5, ((1.27 + 1.23) / 2d) + .15);
        shuttleTimeOfFlightLookupTable.put(7d, 1.37 + .15);
        shuttleTimeOfFlightLookupTable.put(8.5, 1.47 + .15);
        shuttleTimeOfFlightLookupTable.put(10d, ((1.57 + 1.54) / 2d) + .15);

    }


    public static ShotCalculator getInstance() {
        if (shotCalculator == null) shotCalculator = new ShotCalculator();
        return shotCalculator;
    }

    @Override
    public void periodic() {
        if (RobotContainer.getPose() != null) {
            Pose2d estimatedPose = RobotContainer.getPose();
            Pose2d estimatedPosePhaseDelayed =
                    estimatedPose.exp(
                            new Twist2d(
                                    RobotContainer.getSpeeds().vxMetersPerSecond * Constants.PHASE_DELAY
                                            + RobotContainer.getAccelerationX() * Constants.ACCELERATION_PHASE_DELAY,
                                    RobotContainer.getSpeeds().vyMetersPerSecond * Constants.PHASE_DELAY
                                            + RobotContainer.getAccelerationY() * Constants.ACCELERATION_PHASE_DELAY,
                                    RobotContainer.getSpeeds().omegaRadiansPerSecond * Constants.PHASE_DELAY
                                            + RobotContainer.getAccelerationOmega() * Constants.ACCELERATION_PHASE_DELAY));

            Pair<Pose2d, Double> turretPhaseDelayed = createFutureTurretPose(estimatedPosePhaseDelayed,
                    ChassisSpeeds.fromRobotRelativeSpeeds(RobotContainer.getSpeeds(),
                            RobotContainer.getPose().getRotation()));
            Pair<Pose2d, Double> turret = createFutureTurretPose(estimatedPose,
                    ChassisSpeeds.fromRobotRelativeSpeeds(RobotContainer.getSpeeds(),
                            RobotContainer.getPose().getRotation()));

            futureTurretPositionPhaseDelayed = turretPhaseDelayed.getFirst();
            futureTurretToTargetDistancePhaseDelayed = turretPhaseDelayed.getSecond();

            futureTurretPosition = turret.getFirst();
            futureTurretToTargetDistance = turret.getSecond();

            if (RobotContainer.getShotMode() == ShotMode.SHOOTING) {
                hoodAnglePhaseDelayed = hoodLookupTable.get(futureTurretToTargetDistancePhaseDelayed);
                hoodAngle = hoodLookupTable.get(futureTurretToTargetDistance);
            } else {
                hoodAnglePhaseDelayed = shuttleHoodLookupTable.get(futureTurretToTargetDistancePhaseDelayed);
                hoodAngle = shuttleHoodLookupTable.get(futureTurretToTargetDistance);
            }
            if (Double.isNaN(lastHoodAnglePhaseDelayed)) lastHoodAnglePhaseDelayed = hoodAnglePhaseDelayed;
            hoodVelocityPhaseDelayed = (hoodAnglePhaseDelayed - lastHoodAnglePhaseDelayed) / .005;

            if (Double.isNaN(lastHoodAngle)) lastHoodAngle = hoodAngle;
            hoodVelocity = (hoodAngle - lastHoodAngle) / .005;

            if (RobotContainer.getShotMode() == ShotMode.SHOOTING) {
                Translation2d hubCenter = AllianceFlipper.getCorrectAlliance(Constants.BLUE_HUB_CENTER,
                        Constants.RED_HUB_CENTER);

                turretRotationPhaseDelayed = hubCenter
                        .minus(futureTurretPositionPhaseDelayed.getTranslation()).getAngle();
            } else if (RobotContainer.getShotMode() == ShotMode.SHUTTLING_LEFT) {
                Translation2d shuttlingLeftCorner = AllianceFlipper.getCorrectAlliance(
                        Constants.BLUE_SHUTTLE_LEFT_CORNER, Constants.RED_SHUTTLE_LEFT_CORNER);

                turretRotationPhaseDelayed = shuttlingLeftCorner
                        .minus(futureTurretPositionPhaseDelayed.getTranslation()).getAngle();
            } else if (RobotContainer.getShotMode() == ShotMode.SHUTTLING_RIGHT) {
                Translation2d shuttlingRightCorner = AllianceFlipper.getCorrectAlliance(
                        Constants.BLUE_SHUTTLE_RIGHT_CORNER, Constants.RED_SHUTTLE_RIGHT_CORNER);

                turretRotationPhaseDelayed = shuttlingRightCorner
                        .minus(futureTurretPositionPhaseDelayed.getTranslation()).getAngle();
            }
            if (lastTurretRotationPhaseDelayed == null) lastTurretRotationPhaseDelayed = turretRotationPhaseDelayed;
            turretVelocityPhaseDelayed = turretRotationPhaseDelayed
                    .minus(lastTurretRotationPhaseDelayed).getDegrees() / .005;

            // Sets turret angle and then adjusts it based on robot's rotation in relation to target
            turretAnglePhaseDelayed = turretRotationPhaseDelayed.getDegrees();
            turretAnglePhaseDelayed -= futureTurretPositionPhaseDelayed.getRotation().getDegrees();


            //Wraps to within bounds
            while (turretAnglePhaseDelayed <= TurretConstants.MIN) turretAnglePhaseDelayed += 360;
            while (turretAnglePhaseDelayed >= TurretConstants.MAX) turretAnglePhaseDelayed -= 360;

            turretVelocityPhaseDelayed -= (Math.toDegrees(RobotContainer.getSpeeds().omegaRadiansPerSecond));

            //Turret Angle Calculations
            if (RobotContainer.getShotMode() == ShotMode.SHOOTING) {
                Translation2d hubCenter = AllianceFlipper.getCorrectAlliance(Constants.BLUE_HUB_CENTER,
                        Constants.RED_HUB_CENTER);

//                hubCenter = Constants.RED_HUB_FRONT_CENTER;

                turretRotation = hubCenter
                        .minus(futureTurretPosition.getTranslation()).getAngle();
            } else if (RobotContainer.getShotMode() == ShotMode.SHUTTLING_LEFT) {
                Translation2d shuttlingLeftCorner = AllianceFlipper.getCorrectAlliance(
                        Constants.BLUE_SHUTTLE_LEFT_CORNER, Constants.RED_SHUTTLE_LEFT_CORNER);

                turretRotation = shuttlingLeftCorner
                        .minus(futureTurretPosition.getTranslation()).getAngle();
            } else if (RobotContainer.getShotMode() == ShotMode.SHUTTLING_RIGHT) {
                Translation2d shuttlingRightCorner = AllianceFlipper.getCorrectAlliance(
                        Constants.BLUE_SHUTTLE_RIGHT_CORNER, Constants.RED_SHUTTLE_RIGHT_CORNER);

                turretRotation = shuttlingRightCorner
                        .minus(futureTurretPosition.getTranslation()).getAngle();
            }
            if (lastTurretRotation == null) lastTurretRotation = turretRotation;
            turretVelocity = turretRotation
                    .minus(lastTurretRotation).getDegrees() / .005;

            // Sets turret angle and then adjusts it based on robot's rotation in relation to target
            turretAngle = turretRotation.getDegrees();
            turretAngle -= RobotContainer.getPose().getRotation().getDegrees();

            //Wraps to within bounds
            while (turretAngle <= TurretConstants.MIN) turretAngle += 360;
            while (turretAngle >= TurretConstants.MAX) turretAngle -= 360;

            turretVelocity -= (Math.toDegrees(RobotContainer.getSpeeds().omegaRadiansPerSecond));

            // Based on Future Pose
            if (RobotContainer.getShotMode() == ShotMode.SHOOTING) {
                shooterSpeedPhaseDelayed = shooterSpeedLookupTable.get(futureTurretToTargetDistancePhaseDelayed);
                shooterSpeed = shooterSpeedLookupTable.get(futureTurretToTargetDistance);
            } else {
                shooterSpeedPhaseDelayed = shuttleShooterSpeedLookupTable.get(futureTurretToTargetDistancePhaseDelayed);
                shooterSpeed = shuttleShooterSpeedLookupTable.get(futureTurretToTargetDistance);
            }
        }

        lastTurretRotation = turretRotation;
        lastTurretRotationPhaseDelayed = turretRotationPhaseDelayed;
        lastHoodAnglePhaseDelayed = hoodAnglePhaseDelayed;
        lastHoodAngle = hoodAngle;

        // Sets continuous control for turret, shooter, kicker
//        RobotContainer.getTurretControlAuto().setGoal(shotCalculator.getTurretAnglePhaseDelayed(),
//                shotCalculator.getTurretVelocityPhaseDelayed());
        turretSubsystem.updateGoalPosition(getTurretAnglePhaseDelayed(), getTurretVelocityPhaseDelayed());
        RobotContainer.getShooterControlAuto().setGoal(shotCalculator.getShooterSpeedPhaseDelayed());
        RobotContainer.getKickerControlAuto().setGoal(shotCalculator.getKickerSpeedPhaseDelayed());
        RobotContainer.getHoodControlAuto().setGoal(shotCalculator.getHoodAnglePhaseDelayed(),
                shotCalculator.getHoodVelocityPhaseDelayed());

    }


    public Pair<Pose2d, Double> createFutureTurretPose(Pose2d pose2d, ChassisSpeeds fieldRelativeVelocity) {
        Pose2d turretPosition = pose2d.transformBy(Constants.ROBOT_TO_TURRET);

        // Sets distance values for lookup tables based on ShotMode
        double turretToTargetDistance = 0;

        if (RobotContainer.getShotMode() == ShotMode.SHOOTING) {
            Translation2d hubCenter = AllianceFlipper.getCorrectAlliance(Constants.BLUE_HUB_CENTER,
                    Constants.RED_HUB_CENTER);

            turretToTargetDistance = hubCenter
                    .getDistance(turretPosition.getTranslation());
        } else if (RobotContainer.getShotMode() == ShotMode.SHUTTLING_LEFT) {
            Translation2d shuttlingLeftCorner = AllianceFlipper.getCorrectAlliance(
                    Constants.BLUE_SHUTTLE_LEFT_CORNER, Constants.RED_SHUTTLE_LEFT_CORNER);

            turretToTargetDistance = shuttlingLeftCorner
                    .getDistance(turretPosition.getTranslation());
        } else if (RobotContainer.getShotMode() == ShotMode.SHUTTLING_RIGHT) {
            Translation2d shuttlingRightCorner = AllianceFlipper.getCorrectAlliance(
                    Constants.BLUE_SHUTTLE_RIGHT_CORNER, Constants.RED_SHUTTLE_RIGHT_CORNER);

            turretToTargetDistance = shuttlingRightCorner
                    .getDistance(turretPosition.getTranslation());
        }

        //Robot's current velocity
        double robotAngleRadians = pose2d.getRotation().getRadians();

        //Turret's current velocity imparted from robot
        double turretVelocityX =
                fieldRelativeVelocity.vxMetersPerSecond
                        + fieldRelativeVelocity.omegaRadiansPerSecond
                        * (-Constants.ROBOT_TO_TURRET.getY() * Math.cos(robotAngleRadians)
                        - Constants.ROBOT_TO_TURRET.getX() * Math.sin(robotAngleRadians));
        double turretVelocityY =
                fieldRelativeVelocity.vyMetersPerSecond
                        + fieldRelativeVelocity.omegaRadiansPerSecond
                        * (Constants.ROBOT_TO_TURRET.getX() * Math.cos(robotAngleRadians)
                        - Constants.ROBOT_TO_TURRET.getY() * Math.sin(robotAngleRadians));

        //Account for velocity
        double timeOfFlight;
        Pose2d futureTurretPosition = turretPosition;
        double lookaheadTurretToTargetDistance = turretToTargetDistance;

//        Iterate to converge on a final future position
        for (int i = 0; i < 10; i++) {
            timeOfFlight = timeOfFlightLookupTable.get(lookaheadTurretToTargetDistance);
            double offsetX = turretVelocityX * timeOfFlight;
            double offsetY = turretVelocityY * timeOfFlight;
            futureTurretPosition =
                    new Pose2d(
                            turretPosition.getTranslation().plus(new Translation2d(offsetX, offsetY)),
                            turretPosition.getRotation());
            if (RobotContainer.getShotMode() == ShotMode.SHOOTING) {
                Translation2d hubCenter = AllianceFlipper.getCorrectAlliance(Constants.BLUE_HUB_CENTER,
                        Constants.RED_HUB_CENTER);

                lookaheadTurretToTargetDistance = hubCenter
                        .getDistance(futureTurretPosition.getTranslation());
            } else if (RobotContainer.getShotMode() == ShotMode.SHUTTLING_LEFT) {
                Translation2d shuttlingLeftCorner = AllianceFlipper.getCorrectAlliance(
                        Constants.BLUE_SHUTTLE_LEFT_CORNER, Constants.RED_SHUTTLE_LEFT_CORNER);

                lookaheadTurretToTargetDistance = shuttlingLeftCorner
                        .getDistance(futureTurretPosition.getTranslation());
            } else if (RobotContainer.getShotMode() == ShotMode.SHUTTLING_RIGHT) {
                Translation2d shuttlingRightCorner = AllianceFlipper.getCorrectAlliance(
                        Constants.BLUE_SHUTTLE_RIGHT_CORNER, Constants.RED_SHUTTLE_RIGHT_CORNER);

                lookaheadTurretToTargetDistance = shuttlingRightCorner
                        .getDistance(futureTurretPosition.getTranslation());
            }
        }

        return new Pair<>(futureTurretPosition, lookaheadTurretToTargetDistance);
    }

    public double getHoodAngle() {
        return hoodAngle;
    }

    public double getHoodVelocity() {
        return hoodVelocity;
    }

    public double getHoodAnglePhaseDelayed() {
        return hoodAnglePhaseDelayed;
    }

    public double getHoodVelocityPhaseDelayed() {
        return hoodVelocityPhaseDelayed;
    }

    public double getShooterSpeed() {
        return shooterSpeed;
    }

    public double getKickerSpeed() {
        return KickerConstants.INDEXING_SPEED;  //shooterSpeed * KickerConstants.SCALE_FACTOR
    }

    public double getShooterSpeedPhaseDelayed() {
        return shooterSpeedPhaseDelayed;
    }

    public double getKickerSpeedPhaseDelayed() {
        return KickerConstants.INDEXING_SPEED;
    }

    public double getTurretAngle() {
        return turretAngle;
    }

    public double getTurretVelocity() {
        return turretVelocity;
    }

    public double getTurretAnglePhaseDelayed() {
        return turretAnglePhaseDelayed;
    }

    public double getTurretVelocityPhaseDelayed() {
        return turretVelocityPhaseDelayed;
    }

    public double getTurretToTargetDistance() {
        return futureTurretToTargetDistance;
    }

    public Pose2d getFutureTurretPosition() {
        return futureTurretPosition;
    }

    public Pose2d getFutureTurretPositionPhaseDelayed() {
        return futureTurretPositionPhaseDelayed;
    }

    public boolean isWithinBounds() {
        return futureTurretToTargetDistance >= (1.968 + Constants.HUB_RADIUS)
                && futureTurretToTargetDistance <= (5d + Constants.HUB_RADIUS);
    }
}

