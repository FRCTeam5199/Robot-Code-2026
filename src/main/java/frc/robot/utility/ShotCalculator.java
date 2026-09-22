package frc.robot.utility;

import java.util.List;

import org.wpilib.command2.SubsystemBase;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Twist2d;
import org.wpilib.math.interpolation.InterpolatingDoubleTreeMap;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.util.Pair;

import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
import frc.robot.constants.KickerConstants;
import frc.robot.constants.TurretConstants;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class ShotCalculator extends SubsystemBase {
    private static ShotCalculator shotCalculator;
    private Pose2d futureTurretPosition, futureTurretPositionPhaseDelayed;
    private double turretAngle, turretAnglePhaseDelayed;
    // private double lastTurretAngle, lastTurretAnglePhaseDelayed;
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
    // private ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();
    private double futureTurretToTargetDistance, futureTurretToTargetDistancePhaseDelayed = 0;

    public static final List<Double> DISTANCES = List.of(
        1.00, 1.05, 1.10, 1.15, 1.20, 1.25, 1.30, 1.35, 1.40, 1.45, 1.50, 1.55, 1.60, 1.65, 1.70, 1.75, 1.80, 1.85, 1.90, 1.95,
        2.00, 2.05, 2.10, 2.15, 2.20, 2.25, 2.30, 2.35, 2.40, 2.45, 2.50, 2.55, 2.60, 2.65, 2.70, 2.75, 2.80, 2.85, 2.90, 2.95,
        3.00, 3.05, 3.10, 3.15, 3.20, 3.25, 3.30, 3.35, 3.40, 3.45, 3.50, 3.55, 3.60, 3.65, 3.70, 3.75, 3.80, 3.85, 3.90, 3.95,
        4.00, 4.05, 4.10, 4.15, 4.20, 4.25, 4.30, 4.35, 4.40, 4.45, 4.50, 4.55, 4.60, 4.65, 4.70, 4.75, 4.80, 4.85, 4.90, 4.95,
        5.00, 5.05, 5.10, 5.15, 5.20, 5.25, 5.30, 5.35, 5.40, 5.45, 5.50, 5.55, 5.60, 5.65, 5.70, 5.75, 5.80, 5.85, 5.90, 5.95,
        6.00, 6.05, 6.10, 6.15, 6.20, 6.25, 6.30, 6.35, 6.40, 6.45, 6.50, 6.55, 6.60, 6.65, 6.70, 6.75, 6.80, 6.85, 6.90, 6.95,
        7.00, 7.05, 7.10, 7.15, 7.20, 7.25, 7.30, 7.35, 7.40, 7.45, 7.50, 7.55, 7.60, 7.65, 7.70, 7.75, 7.80, 7.85, 7.90, 7.95, 8.00);
 
    public static final List<Double> HOOD_ANGLES = List.of(
        0.0, 1.0, 1.0, 2.0, 2.0, 3.0, 3.0, 4.0, 4.0, 5.0, 5.0, 5.0, 6.0, 6.0, 6.0, 7.0, 7.0, 8.0, 8.0, 8.0,
        9.0, 11.0, 13.0, 13.0, 13.0, 13.0, 13.0, 16.0, 16.0, 15.0, 15.0, 15.0, 15.0, 15.0, 18.0, 17.0, 17.0, 17.0, 17.0, 16.0,
        16.0, 19.0, 18.0, 18.0, 18.0, 18.0, 17.0, 20.0, 20.0, 19.0, 19.0, 19.0, 18.0, 21.0, 21.0, 20.0, 20.0, 19.0, 19.0, 22.0,
        21.0, 21.0, 20.0, 20.0, 22.0, 22.0, 21.0, 21.0, 20.0, 22.0, 22.0, 22.0, 21.0, 21.0, 23.0, 23.0, 22.0, 22.0, 21.0, 23.0,
        23.0, 22.0, 22.0, 23.0, 24.0, 23.0, 22.0, 24.0, 23.0, 24.0, 23.0, 22.0, 24.0, 24.0, 23.0, 23.0, 24.0, 24.0, 24.0, 23.0,
        24.0, 25.0, 24.0, 23.0, 24.0, 25.0, 24.0, 23.0, 24.0, 25.0, 24.0, 25.0, 24.0, 25.0, 24.0, 25.0, 24.0, 25.0, 24.0, 25.0,
        24.0, 25.0, 24.0, 25.0, 25.0, 25.0, 25.0, 25.0, 25.0, 25.0, 25.0, 25.0, 25.0, 24.0, 25.0, 25.0, 25.0, 25.0, 25.0, 25.0, 25.0);
 
    public static final List<Double> SHOOTER_SPEEDS = List.of(
        38.5, 38.5, 38.8, 39.1, 39.4, 39.4, 39.6, 39.6, 40.2, 40.2, 40.5, 41.0, 41.1, 41.4, 41.7, 41.7, 42.0, 42.1, 42.6, 42.9,
        42.9, 42.6, 42.6, 42.9, 43.2, 43.5, 43.8, 43.8, 44.0, 44.3, 44.6, 44.9, 45.2, 45.5, 45.5, 45.8, 46.1, 46.4, 46.7, 47.0,
        47.3, 47.3, 47.6, 47.9, 48.1, 48.4, 48.7, 48.9, 49.0, 49.3, 49.6, 49.9, 50.2, 50.3, 50.5, 50.8, 51.1, 51.4, 51.7, 51.7,
        52.0, 52.2, 52.5, 52.8, 53.0, 53.1, 53.4, 53.7, 54.0, 54.2, 54.3, 54.6, 54.9, 55.2, 55.3, 55.5, 55.8, 56.1, 56.3, 56.5,
        56.6, 56.9, 57.2, 57.4, 57.5, 57.8, 58.1, 58.3, 58.5, 58.7, 59.0, 59.3, 59.4, 59.6, 59.9, 60.2, 60.3, 60.4, 60.7, 61.0,
        61.2, 61.3, 61.6, 61.9, 62.1, 62.2, 62.5, 62.8, 62.9, 63.1, 63.4, 63.5, 63.8, 64.0, 64.3, 64.4, 64.7, 64.8, 65.1, 65.3,
        65.6, 65.7, 66.0, 66.2, 66.3, 66.6, 66.7, 67.0, 67.2, 67.5, 67.6, 67.9, 68.1, 68.4, 68.5, 68.7, 68.9, 69.1, 69.4, 69.5, 69.8);
 
    public static final List<Double> TIME_OF_FLIGHT = List.of(
        0.711, 0.701, 0.729, 0.713, 0.74, 0.729, 0.753, 0.742, 0.759, 0.749, 0.77, 0.787, 0.773, 0.792, 0.811, 0.799, 0.817, 0.802, 0.816, 0.832,
        0.82, 0.785, 0.75, 0.763, 0.776, 0.789, 0.802, 0.746, 0.757, 0.792, 0.804, 0.815, 0.826, 0.838, 0.782, 0.815, 0.825, 0.835, 0.845, 0.881,
        0.891, 0.833, 0.865, 0.874, 0.883, 0.892, 0.927, 0.866, 0.877, 0.908, 0.917, 0.925, 0.959, 0.897, 0.907, 0.938, 0.946, 0.979, 0.987, 0.927,
        0.958, 0.965, 0.997, 1.004, 0.965, 0.975, 1.006, 1.013, 1.046, 1.004, 1.014, 1.02, 1.052, 1.059, 1.018, 1.027, 1.057, 1.064, 1.097, 1.054,
        1.062, 1.094, 1.1, 1.083, 1.067, 1.097, 1.129, 1.086, 1.117, 1.1, 1.131, 1.163, 1.118, 1.126, 1.158, 1.163, 1.145, 1.153, 1.158, 1.19,
        1.171, 1.153, 1.184, 1.216, 1.196, 1.178, 1.209, 1.242, 1.221, 1.202, 1.234, 1.214, 1.246, 1.226, 1.258, 1.238, 1.27, 1.249, 1.282, 1.261,
        1.294, 1.272, 1.306, 1.284, 1.291, 1.295, 1.302, 1.306, 1.313, 1.317, 1.324, 1.329, 1.335, 1.37, 1.346, 1.353, 1.357, 1.364, 1.368, 1.375, 1.379);

    private ShotCalculator() {
        //Motor Rotations = degrees / 360 / .01255707762557077625570776255708
        //Degrees = motorRot * 360 * .01255707762557077625570776255708
        hoodLookupTable = new InterpolatingDoubleTreeMap();
        shooterSpeedLookupTable = new InterpolatingDoubleTreeMap();
        timeOfFlightLookupTable = new InterpolatingDoubleTreeMap();

        shuttleHoodLookupTable = new InterpolatingDoubleTreeMap();
        shuttleShooterSpeedLookupTable = new InterpolatingDoubleTreeMap();
        shuttleTimeOfFlightLookupTable = new InterpolatingDoubleTreeMap();
        
        for(int i=0; i<DISTANCES.size(); i++) {
            hoodLookupTable.put(DISTANCES.get(i), HOOD_ANGLES.get(i));
        }

        for(int i=0; i<DISTANCES.size(); i++) {
            shooterSpeedLookupTable.put(DISTANCES.get(i), SHOOTER_SPEEDS.get(i));
        }

        for(int i=0; i<DISTANCES.size(); i++) {
            timeOfFlightLookupTable.put(DISTANCES.get(i), TIME_OF_FLIGHT.get(i));
        }

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

        // lastTurretAngle = turretSubsystem.getDegrees();
        // lastTurretAnglePhaseDelayed = turretSubsystem.getDegrees();
    }


    public static ShotCalculator getInstance() {
        if (shotCalculator == null) shotCalculator = new ShotCalculator();
        return shotCalculator;
    }

    @Override
    public void periodic() {
        if (RobotContainer.getPose() != null) {
            Pose2d estimatedPose = RobotContainer.getPose();
            Pose2d estimatedPosePhaseDelayed = estimatedPose.plus(new Twist2d(
                RobotContainer.getVelocity().vx * Constants.PHASE_DELAY + RobotContainer.getAccelerationX() * Constants.ACCELERATION_PHASE_DELAY,
                RobotContainer.getVelocity().vy * Constants.PHASE_DELAY + RobotContainer.getAccelerationY() * Constants.ACCELERATION_PHASE_DELAY,
                RobotContainer.getVelocity().omega * Constants.PHASE_DELAY + RobotContainer.getAccelerationOmega() * Constants.ACCELERATION_PHASE_DELAY).exp());

            // TODO: Test
            Pair<Pose2d, Double> turretPhaseDelayed = createFutureTurretPose(estimatedPosePhaseDelayed, new ChassisVelocities(
                new Translation2d(RobotContainer.getVelocity().vx, RobotContainer.getVelocity().vy).rotateBy(RobotContainer.getPose().getRotation()).getX(),
                new Translation2d(RobotContainer.getVelocity().vx, RobotContainer.getVelocity().vy).rotateBy(RobotContainer.getPose().getRotation()).getY(),
                RobotContainer.getVelocity().omega));
            Pair<Pose2d, Double> turret = createFutureTurretPose(estimatedPose, new ChassisVelocities(
                new Translation2d(RobotContainer.getVelocity().vx, RobotContainer.getVelocity().vy).rotateBy(RobotContainer.getPose().getRotation()).getX(),
                new Translation2d(RobotContainer.getVelocity().vx, RobotContainer.getVelocity().vy).rotateBy(RobotContainer.getPose().getRotation()).getY(),
                RobotContainer.getVelocity().omega));

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
                Translation2d hubCenter = AllianceFlipper.getCorrectAlliance(Constants.BLUE_HUB_CENTER, Constants.RED_HUB_CENTER);
                turretRotationPhaseDelayed = hubCenter.minus(futureTurretPositionPhaseDelayed.getTranslation()).getAngle().get();
            } else if (RobotContainer.getShotMode() == ShotMode.SHUTTLING_LEFT) {
                Translation2d shuttlingLeftCorner = AllianceFlipper.getCorrectAlliance(Constants.BLUE_SHUTTLE_LEFT_CORNER, Constants.RED_SHUTTLE_LEFT_CORNER);
                turretRotationPhaseDelayed = shuttlingLeftCorner.minus(futureTurretPositionPhaseDelayed.getTranslation()).getAngle().get();
            } else if (RobotContainer.getShotMode() == ShotMode.SHUTTLING_RIGHT) {
                Translation2d shuttlingRightCorner = AllianceFlipper.getCorrectAlliance(Constants.BLUE_SHUTTLE_RIGHT_CORNER, Constants.RED_SHUTTLE_RIGHT_CORNER);
                turretRotationPhaseDelayed = shuttlingRightCorner.minus(futureTurretPositionPhaseDelayed.getTranslation()).getAngle().get();
            }

            if (lastTurretRotationPhaseDelayed == null) lastTurretRotationPhaseDelayed = turretRotationPhaseDelayed;
            turretVelocityPhaseDelayed = turretRotationPhaseDelayed.minus(lastTurretRotationPhaseDelayed).getDegrees() / .005;

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

            turretVelocityPhaseDelayed -= (Math.toDegrees(RobotContainer.getVelocity().omega));

            //Turret Angle Calculations
            if (RobotContainer.getShotMode() == ShotMode.SHOOTING) {
                Translation2d hubCenter = AllianceFlipper.getCorrectAlliance(Constants.BLUE_HUB_CENTER,
                        Constants.RED_HUB_CENTER);

            // hubCenter = Constants.RED_HUB_FRONT_CENTER;

                turretRotation = hubCenter.minus(futureTurretPosition.getTranslation()).getAngle().get();
            } else if (RobotContainer.getShotMode() == ShotMode.SHUTTLING_LEFT) {
                Translation2d shuttlingLeftCorner = AllianceFlipper.getCorrectAlliance(
                        Constants.BLUE_SHUTTLE_LEFT_CORNER, Constants.RED_SHUTTLE_LEFT_CORNER);

                turretRotation = shuttlingLeftCorner.minus(futureTurretPosition.getTranslation()).getAngle().get();
            } else if (RobotContainer.getShotMode() == ShotMode.SHUTTLING_RIGHT) {
                Translation2d shuttlingRightCorner = AllianceFlipper.getCorrectAlliance(
                        Constants.BLUE_SHUTTLE_RIGHT_CORNER, Constants.RED_SHUTTLE_RIGHT_CORNER);

                turretRotation = shuttlingRightCorner.minus(futureTurretPosition.getTranslation()).getAngle().get();
            }

            if (lastTurretRotation == null) lastTurretRotation = turretRotation;
            turretVelocity = turretRotation.minus(lastTurretRotation).getDegrees() / .005;

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

            turretVelocity -= (Math.toDegrees(RobotContainer.getVelocity().omega));

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


    public Pair<Pose2d, Double> createFutureTurretPose(Pose2d pose2d, ChassisVelocities fieldRelativeVelocity) {
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
                fieldRelativeVelocity.vx
                        + fieldRelativeVelocity.omega
                        * (-Constants.ROBOT_TO_TURRET.getY() * Math.cos(robotAngleRadians)
                        - Constants.ROBOT_TO_TURRET.getX() * Math.sin(robotAngleRadians));
        double turretVelocityY =
                fieldRelativeVelocity.vy
                        + fieldRelativeVelocity.omega
                        * (Constants.ROBOT_TO_TURRET.getX() * Math.cos(robotAngleRadians)
                        - Constants.ROBOT_TO_TURRET.getY() * Math.sin(robotAngleRadians));

        // Account for velocity
        double timeOfFlight;
        Pose2d futureTurretPosition = turretPosition;
        double lookaheadTurretToTargetDistance = turretToTargetDistance;

        // Iterate to converge on a final future position
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

