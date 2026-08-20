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
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.TurretConstants;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
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
    private double lastTurretAngle, lastTurretAnglePhaseDelayed;
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
    private HoodSubsystem hoodSubsystem = HoodSubsystem.getInstance();
    private ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();
    private double futureTurretToTargetDistance, futureTurretToTargetDistancePhaseDelayed = 0;

    private ShotCalculator() {
        //Motor Rotations = degrees / 360 / .01255707762557077625570776255708
        //Degrees = motorRot * 360 * .01255707762557077625570776255708
        hoodLookupTable = new InterpolatingDoubleTreeMap();
        shooterSpeedLookupTable = new InterpolatingDoubleTreeMap();
        timeOfFlightLookupTable = new InterpolatingDoubleTreeMap();

        shuttleHoodLookupTable = new InterpolatingDoubleTreeMap();
        shuttleShooterSpeedLookupTable = new InterpolatingDoubleTreeMap();
        shuttleTimeOfFlightLookupTable = new InterpolatingDoubleTreeMap();

        hoodLookupTable.put(1.01 + Constants.HUB_RADIUS, 4d);
        hoodLookupTable.put(1.528 + Constants.HUB_RADIUS, 6d);
        hoodLookupTable.put(2.05 + Constants.HUB_RADIUS, 8d);
        hoodLookupTable.put(2.466 + Constants.HUB_RADIUS, 10d);
        hoodLookupTable.put(2.97 + Constants.HUB_RADIUS, 12d);
        hoodLookupTable.put(3.517 + Constants.HUB_RADIUS, 13d);
        hoodLookupTable.put(3.968 + Constants.HUB_RADIUS, 15d);
        hoodLookupTable.put(4.522 + Constants.HUB_RADIUS, 17d);
        hoodLookupTable.put(5.074 + Constants.HUB_RADIUS, 19d);
        hoodLookupTable.put(5.474 + Constants.HUB_RADIUS, 21d);

        shooterSpeedLookupTable.put(1.01 + Constants.HUB_RADIUS, 42d);
        shooterSpeedLookupTable.put(1.528 + Constants.HUB_RADIUS, 45d);
        shooterSpeedLookupTable.put(2.05 + Constants.HUB_RADIUS, 48d);
        shooterSpeedLookupTable.put(2.466 + Constants.HUB_RADIUS, 50.5);
        shooterSpeedLookupTable.put(2.97 + Constants.HUB_RADIUS, 54d);
        shooterSpeedLookupTable.put(3.517 + Constants.HUB_RADIUS, 57d);
        shooterSpeedLookupTable.put(3.968 + Constants.HUB_RADIUS, 60d);
        shooterSpeedLookupTable.put(4.522 + Constants.HUB_RADIUS, 63d);
        shooterSpeedLookupTable.put(5.074 + Constants.HUB_RADIUS, 66d);
        shooterSpeedLookupTable.put(5.474 + Constants.HUB_RADIUS, 69d);

        //New table done right before going to Code Orange
        timeOfFlightLookupTable.put(1.01 + Constants.HUB_RADIUS, .95);
        timeOfFlightLookupTable.put(1.528 + Constants.HUB_RADIUS, .94);
        timeOfFlightLookupTable.put(2.05 + Constants.HUB_RADIUS, 1.03);
        timeOfFlightLookupTable.put(2.466 + Constants.HUB_RADIUS, 1.01);
        timeOfFlightLookupTable.put(2.97 + Constants.HUB_RADIUS, 1.04);
        timeOfFlightLookupTable.put(3.517 + Constants.HUB_RADIUS, 1.197);
        timeOfFlightLookupTable.put(3.968 + Constants.HUB_RADIUS, 1.10);
        timeOfFlightLookupTable.put(4.522 + Constants.HUB_RADIUS, 1.15);
        timeOfFlightLookupTable.put(5.074 + Constants.HUB_RADIUS, 1.25);
        timeOfFlightLookupTable.put(5.474 + Constants.HUB_RADIUS, 1.16);

        //----------------
        shuttleHoodLookupTable.put(4.726098, 10d + 2d + 2d);
        shuttleHoodLookupTable.put(7.415803, 16d + 2d + 2d);
        shuttleHoodLookupTable.put(10.540514, 24d);
        shuttleHoodLookupTable.put(10.970932, 24d);

        shuttleShooterSpeedLookupTable.put(4.726098, 50d - 5d);
        shuttleShooterSpeedLookupTable.put(7.415803, 79d - 5d);
        shuttleShooterSpeedLookupTable.put(10.540514, 100d);
        shuttleShooterSpeedLookupTable.put(10.970932, 100d);
        shuttleShooterSpeedLookupTable.put(12d, 100d);

        shuttleTimeOfFlightLookupTable.put(4.726098, 1.37 + .075);
        shuttleTimeOfFlightLookupTable.put(7.415803, 1.46 + .075);
        shuttleTimeOfFlightLookupTable.put(10.540514, 1.55 + .075);
        shuttleTimeOfFlightLookupTable.put(10.970932, 1.56 + .075);

        lastTurretAngle = turretSubsystem.getDegrees();
        lastTurretAnglePhaseDelayed = turretSubsystem.getDegrees();
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
            double[] turretAngles = new double[]{turretAnglePhaseDelayed - 360,
                    turretAnglePhaseDelayed, turretAnglePhaseDelayed + 360};
            int smallestValidDistanceIndex = -1;
            double smallestDistance = 99999d;
            for (int i = 0; i < 3; i++) {
                double angle = turretAngles[i];
                if (angle <= TurretConstants.MIN || angle >= TurretConstants.MAX) continue;
                if (Math.abs(angle - turretSubsystem.getDegrees()) < smallestDistance) {
                    smallestDistance = Math.abs(angle - turretSubsystem.getDegrees());
                    smallestValidDistanceIndex = i;
                }
            }
            if (smallestValidDistanceIndex == -1) turretAnglePhaseDelayed = turretAngles[1];
            else turretAnglePhaseDelayed = turretAngles[smallestValidDistanceIndex];

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
            turretAngles = new double[]{turretAngle - 360, turretAngle, turretAngle + 360};
            smallestValidDistanceIndex = -1;
            smallestDistance = 99999d;
            for (int i = 0; i < 3; i++) {
                double angle = turretAngles[i];
                if (angle <= TurretConstants.MIN || angle >= TurretConstants.MAX) continue;
                if (Math.abs(angle - turretSubsystem.getDegrees()) < smallestDistance) {
                    smallestDistance = Math.abs(angle - turretSubsystem.getDegrees());
                    smallestValidDistanceIndex = i;
                }
            }
            if (smallestValidDistanceIndex == -1) turretAngle = turretAngles[1];
            else turretAngle = turretAngles[smallestValidDistanceIndex];

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
        RobotContainer.getShooterControlAutoRB().setGoal(shotCalculator.getShooterSpeedPhaseDelayed());
        RobotContainer.getKickerControlAuto().setGoal(shotCalculator.getKickerSpeedPhaseDelayed());
//        shooterSubsystem.setVelocity(shotCalculator.getShooterSpeedPhaseDelayed());
//        RobotContainer.getHoodControlAuto().setGoal(shotCalculator.getHoodAnglePhaseDelayed(),
//                shotCalculator.getHoodVelocityPhaseDelayed());
        hoodSubsystem.updateGoalPosition(getHoodAnglePhaseDelayed(), getHoodVelocityPhaseDelayed());

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
        if (RobotContainer.getShotMode() != ShotMode.SHOOTING) return true;
        return futureTurretToTargetDistance >= (.997 + Constants.HUB_RADIUS)
                && futureTurretToTargetDistance <= (5d + Constants.HUB_RADIUS);
    }
}

