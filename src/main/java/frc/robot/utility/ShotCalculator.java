package frc.robot.utility;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
import frc.robot.constants.TurretConstants;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.TurretSubsystem;

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

        //key - distance to front center hub
        hoodLookupTable.put(1.01, 0d);                   //motor rotation * 360 * gear ratio
        hoodLookupTable.put(2.06375, 0d);
        hoodLookupTable.put(3.0051375, 4.9999999999999968);
//        hoodLookupTable.put(3.52425, 7.4999999952);
        hoodLookupTable.put(4.03606, 8.74999999944);
        hoodLookupTable.put(4.4973875, 9.9999999999936);
        hoodLookupTable.put(5.038, 11.2499999928);
        hoodLookupTable.put(5.554, 13.749999912);
        hoodLookupTable.put(5.9737625, 14.9999999904);
        hoodLookupTable.put(6.453, 16.2499999896);     // We are here

        shooterSpeedLookupTable.put(1.01, 27d - 2);
        shooterSpeedLookupTable.put(2.06375, 31d - 2);
        shooterSpeedLookupTable.put(3.0051375, 32.75 - 1.5);
//        shooterSpeedLookupTable.put(3.5, 33.8 - 1.5);
        shooterSpeedLookupTable.put(4.03606, 35.2 - 1.5);
        shooterSpeedLookupTable.put(4.4973875, 36d - 1.5);
        shooterSpeedLookupTable.put(5.038, 38d - 1.5);
        shooterSpeedLookupTable.put(5.554, 40.2 - 1.5);
        shooterSpeedLookupTable.put(5.9737625, 42d - 1.5);
        shooterSpeedLookupTable.put(6.5d, 46d - 1.5);


        timeOfFlightLookupTable.put(1.01, 0.835);
        timeOfFlightLookupTable.put(2.06375, (1.04 + 1.05) / 2d);
        timeOfFlightLookupTable.put(3.0051375, (1.06 + 1.09) / 2d);
//        timeOfFlightLookupTable.put(3.5, (1.11 + 1.16) / 2d);
        timeOfFlightLookupTable.put(4.03606, (1.15 + 1.19) / 2d);
        timeOfFlightLookupTable.put(4.4973875, (1.13 + 1.2) / 2d);
        timeOfFlightLookupTable.put(5.038, (1.25 + 1.27) / 2d);
        timeOfFlightLookupTable.put(5.554, (1.24 + 1.33) / 2d);
        timeOfFlightLookupTable.put(5.9737625, (1.32 + 1.34) / 2d);
        timeOfFlightLookupTable.put(6.5d, (1.32 + 1.3) / 2d);

        //------------------------------------------------------
        shuttleHoodLookupTable.put(5.5, 10d);
        shuttleHoodLookupTable.put(7d, 12d);
        shuttleHoodLookupTable.put(8.5d, 14d);
        shuttleHoodLookupTable.put(10d, 18d);
        shuttleHoodLookupTable.put(14d, 24d);

        shuttleShooterSpeedLookupTable.put(5.5, 30d);
        shuttleShooterSpeedLookupTable.put(7d, 37d);
        shuttleShooterSpeedLookupTable.put(8.5d, 39d);
        shuttleShooterSpeedLookupTable.put(14d, 60d);

        shuttleTimeOfFlightLookupTable.put(5.5, (1.27 + 1.23) / 2d);
        shuttleTimeOfFlightLookupTable.put(7d, 1.37);
        shuttleTimeOfFlightLookupTable.put(8.5, 1.47);
        shuttleTimeOfFlightLookupTable.put(10d, (1.57 + 1.54) / 2d);
        shuttleTimeOfFlightLookupTable.put(14d, (1.61 + 1.71) / 2d);

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
                                    RobotContainer.getRequestXVelocity() * Constants.PHASE_DELAY,
                                    RobotContainer.getRequestYVelocity() * Constants.PHASE_DELAY,
                                    RobotContainer.getRequestRotationalVelocity() * Constants.PHASE_DELAY));

            Pair<Pose2d, Double> turretPhaseDelayed = createFutureTurretPose(estimatedPosePhaseDelayed,
                    new ChassisSpeeds(RobotContainer.getRequestXVelocity(), RobotContainer.getRequestYVelocity(),
                            RobotContainer.getRequestRotationalVelocity()));
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
//            hoodVelocityPhaseDelayed = (hoodAnglePhaseDelayed - HoodSubsystem.getInstance().getDegrees()) / .02;

            if (Double.isNaN(lastHoodAngle)) lastHoodAngle = hoodAngle;
            hoodVelocity = (hoodAngle - lastHoodAngle) / .02;


            if (RobotContainer.getShotMode() == ShotMode.SHOOTING) {
                Translation2d hubCenter = AllianceFlipper.getCorrectAlliance(Constants.BLUE_HUB_CENTER,
                        Constants.RED_HUB_CENTER);
                hubCenter = Constants.RED_HUB_FRONT_CENTER;

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

            double degrees = TurretSubsystem.getInstance().getDegrees() + RobotContainer.getPose().getRotation().getDegrees();
            while (degrees >= 180d) degrees -= 360d;
            while (degrees <= -180d) degrees += 360d;

//            if (Math.abs(TurretSubsystem.getInstance().getDegrees()
//                    + RobotContainer.getPose().getRotation().getDegrees()
//                    - turretRotationPhaseDelayed.getDegrees()) > 10) {
//                degrees = lastTurretRotationPhaseDelayed.getDegrees();
//            }

            turretVelocityPhaseDelayed = turretRotationPhaseDelayed
                    .minus(new Rotation2d(Math.toRadians(degrees))).getDegrees() / .02;
//            System.out.println("Corrected: " + TurretSubsystem.getInstance().getMotorRotFromDegrees(turretVelocityPhaseDelayed));
//            System.out.println("Old: " + TurretSubsystem.getInstance().getMotorRotFromDegrees(turretRotationPhaseDelayed
//                    .minus(lastTurretRotationPhaseDelayed).getDegrees() / .02));
//            turretVelocityPhaseDelayed = turretRotationPhaseDelayed
//                    .minus(lastTurretRotationPhaseDelayed).getDegrees() / .02;

//            System.out.println("Adjusted Velocity: " + turretVelocityPhaseDelayed);
//            System.out.println("Previous Velocity: " + turretRotationPhaseDelayed
//                    .minus(lastTurretRotationPhaseDelayed).getDegrees() / .02);

            // Sets turret angle and then adjusts it based on robot's rotation in relation to target
            turretAnglePhaseDelayed = turretRotationPhaseDelayed.getDegrees();
            turretAnglePhaseDelayed -= futureTurretPositionPhaseDelayed.getRotation().getDegrees();


            //Wraps to within bounds
            while (turretAnglePhaseDelayed <= TurretConstants.MIN) turretAnglePhaseDelayed += 360;
            while (turretAnglePhaseDelayed >= TurretConstants.MAX) turretAnglePhaseDelayed -= 360;

            turretVelocityPhaseDelayed -= (RobotContainer.getRequestRotationalVelocity() / Math.PI * 180d);

            //Turret Angle Calculations
            if (RobotContainer.getShotMode() == ShotMode.SHOOTING) {
                Translation2d hubCenter = AllianceFlipper.getCorrectAlliance(Constants.BLUE_HUB_CENTER,
                        Constants.RED_HUB_CENTER);
                hubCenter = Constants.RED_HUB_FRONT_CENTER;

                turretRotation = hubCenter
                        .minus(futureTurretPositionPhaseDelayed.getTranslation()).getAngle();
            } else if (RobotContainer.getShotMode() == ShotMode.SHUTTLING_LEFT) {
                Translation2d shuttlingLeftCorner = AllianceFlipper.getCorrectAlliance(
                        Constants.BLUE_SHUTTLE_LEFT_CORNER, Constants.RED_SHUTTLE_LEFT_CORNER);

                turretRotation = shuttlingLeftCorner
                        .minus(futureTurretPositionPhaseDelayed.getTranslation()).getAngle();
            } else if (RobotContainer.getShotMode() == ShotMode.SHUTTLING_RIGHT) {
                Translation2d shuttlingRightCorner = AllianceFlipper.getCorrectAlliance(
                        Constants.BLUE_SHUTTLE_RIGHT_CORNER, Constants.RED_SHUTTLE_RIGHT_CORNER);

                turretRotation = shuttlingRightCorner
                        .minus(futureTurretPositionPhaseDelayed.getTranslation()).getAngle();
            }
            if (lastTurretRotation == null) lastTurretRotation = turretRotation;
            turretVelocity = turretRotation
                    .minus(lastTurretRotation).getDegrees() / .02;

            // Sets turret angle and then adjusts it based on robot's rotation in relation to target
            turretAngle = turretRotation.getDegrees();
            turretAngle -= RobotContainer.getPose().getRotation().getDegrees();

            //Wraps to within bounds
            while (turretAngle <= TurretConstants.MIN) turretAngle += 360;
            while (turretAngle >= TurretConstants.MAX) turretAngle -= 360;

            turretVelocity -= (RobotContainer.getSpeeds().omegaRadiansPerSecond / Math.PI * 180d);

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
        RobotContainer.getTurretControlAuto().setGoal(shotCalculator.getTurretAnglePhaseDelayed(),
                shotCalculator.getTurretVelocityPhaseDelayed());
//        RobotContainer.getShooterControlAuto().setGoal(shotCalculator.getShooterSpeedPhaseDelayed());
        RobotContainer.getKickerControlAuto().setGoal(shotCalculator.getKickerSpeedPhaseDelayed());
//        RobotContainer.getHoodControlAuto().setGoal(shotCalculator.getHoodAngle(),
//                0);
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
                        * (Constants.ROBOT_TO_TURRET.getY() * Math.cos(robotAngleRadians)
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

        //Iterate to converge on a final future position
//        for (int i = 0; i < 10; i++) {
//            timeOfFlight = timeOfFlightLookupTable.get(lookaheadTurretToTargetDistance);
//            double offsetX = turretVelocityX * timeOfFlight;
//            double offsetY = turretVelocityY * timeOfFlight;
//            futureTurretPosition =
//                    new Pose2d(
//                            turretPosition.getTranslation().plus(new Translation2d(offsetX, offsetY)),
//                            turretPosition.getRotation());
//            if (RobotContainer.getShotMode() == ShotMode.SHOOTING) {
//                Translation2d hubCenter = AllianceFlipper.getCorrectAlliance(Constants.BLUE_HUB_CENTER,
//                        Constants.RED_HUB_CENTER);
//
//                lookaheadTurretToTargetDistance = hubCenter
//                        .getDistance(turretPosition.getTranslation());
//            } else if (RobotContainer.getShotMode() == ShotMode.SHUTTLING_LEFT) {
//                Translation2d shuttlingLeftCorner = AllianceFlipper.getCorrectAlliance(
//                        Constants.BLUE_SHUTTLE_LEFT_CORNER, Constants.RED_SHUTTLE_LEFT_CORNER);
//
//                lookaheadTurretToTargetDistance = shuttlingLeftCorner
//                        .getDistance(turretPosition.getTranslation());
//            } else if (RobotContainer.getShotMode() == ShotMode.SHUTTLING_RIGHT) {
//                Translation2d shuttlingRightCorner = AllianceFlipper.getCorrectAlliance(
//                        Constants.BLUE_SHUTTLE_RIGHT_CORNER, Constants.RED_SHUTTLE_RIGHT_CORNER);
//
//                lookaheadTurretToTargetDistance = shuttlingRightCorner
//                        .getDistance(turretPosition.getTranslation());
//            }
//        }

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
        return (shooterSpeed / 2d);
    }

    public double getShooterSpeedPhaseDelayed() {
        return shooterSpeedPhaseDelayed;
    }

    public double getKickerSpeedPhaseDelayed() {
        return (shooterSpeedPhaseDelayed / 2d);
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
}

