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
import frc.robot.constants.KickerConstants;
import frc.robot.constants.TurretConstants;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class ShotCalculator {
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
    private static final double CONVERGENCE_EPSILON_METERS = 0.005;

    private ShotCalculator() {
        //Motor Rotations = degrees / 360 / .01255707762557077625570776255708
        //Degrees = motorRot * 360 * .01255707762557077625570776255708
        hoodLookupTable = new InterpolatingDoubleTreeMap();
        shooterSpeedLookupTable = new InterpolatingDoubleTreeMap();
        timeOfFlightLookupTable = new InterpolatingDoubleTreeMap();

        shuttleHoodLookupTable = new InterpolatingDoubleTreeMap();
        shuttleShooterSpeedLookupTable = new InterpolatingDoubleTreeMap();
        shuttleTimeOfFlightLookupTable = new InterpolatingDoubleTreeMap();

        hoodLookupTable.put(1.00, 0d);
        hoodLookupTable.put(1.05, 1d);
        hoodLookupTable.put(1.10, 1d);
        hoodLookupTable.put(1.15, 2d);
        hoodLookupTable.put(1.20, 2d);
        hoodLookupTable.put(1.25, 3d);
        hoodLookupTable.put(1.30, 3d);
        hoodLookupTable.put(1.35, 4d);
        hoodLookupTable.put(1.40, 4d);
        hoodLookupTable.put(1.45, 5d);
        hoodLookupTable.put(1.50, 5d);
        hoodLookupTable.put(1.55, 5d);
        hoodLookupTable.put(1.60, 6d);
        hoodLookupTable.put(1.65, 6d);
        hoodLookupTable.put(1.70, 6d);
        hoodLookupTable.put(1.75, 7d);
        hoodLookupTable.put(1.80, 7d);
        hoodLookupTable.put(1.85, 8d);
        hoodLookupTable.put(1.90, 8d);
        hoodLookupTable.put(1.95, 8d);
        hoodLookupTable.put(2.00, 9d);
        hoodLookupTable.put(2.05, 11d);
        hoodLookupTable.put(2.10, 13d);
        hoodLookupTable.put(2.15, 13d);
        hoodLookupTable.put(2.20, 13d);
        hoodLookupTable.put(2.25, 13d);
        hoodLookupTable.put(2.30, 13d);
        hoodLookupTable.put(2.35, 16d);
        hoodLookupTable.put(2.40, 16d);
        hoodLookupTable.put(2.45, 15d);
        hoodLookupTable.put(2.50, 15d);
        hoodLookupTable.put(2.55, 15d);
        hoodLookupTable.put(2.60, 15d);
        hoodLookupTable.put(2.65, 15d);
        hoodLookupTable.put(2.70, 18d);
        hoodLookupTable.put(2.75, 17d);
        hoodLookupTable.put(2.80, 17d);
        hoodLookupTable.put(2.85, 17d);
        hoodLookupTable.put(2.90, 17d);
        hoodLookupTable.put(2.95, 16d);
        hoodLookupTable.put(3.00, 16d);
        hoodLookupTable.put(3.05, 19d);
        hoodLookupTable.put(3.10, 18d);
        hoodLookupTable.put(3.15, 18d);
        hoodLookupTable.put(3.20, 18d);
        hoodLookupTable.put(3.25, 18d);
        hoodLookupTable.put(3.30, 17d);
        hoodLookupTable.put(3.35, 20d);
        hoodLookupTable.put(3.40, 20d);
        hoodLookupTable.put(3.45, 19d);
        hoodLookupTable.put(3.50, 19d);
        hoodLookupTable.put(3.55, 19d);
        hoodLookupTable.put(3.60, 18d);
        hoodLookupTable.put(3.65, 21d);
        hoodLookupTable.put(3.70, 21d);
        hoodLookupTable.put(3.75, 20d);
        hoodLookupTable.put(3.80, 20d);
        hoodLookupTable.put(3.85, 19d);
        hoodLookupTable.put(3.90, 19d);
        hoodLookupTable.put(3.95, 22d);
        hoodLookupTable.put(4.00, 21d);
        hoodLookupTable.put(4.05, 21d);
        hoodLookupTable.put(4.10, 20d);
        hoodLookupTable.put(4.15, 20d);
        hoodLookupTable.put(4.20, 22d);
        hoodLookupTable.put(4.25, 22d);
        hoodLookupTable.put(4.30, 21d);
        hoodLookupTable.put(4.35, 21d);
        hoodLookupTable.put(4.40, 20d);
        hoodLookupTable.put(4.45, 22d);
        hoodLookupTable.put(4.50, 22d);
        hoodLookupTable.put(4.55, 22d);
        hoodLookupTable.put(4.60, 21d);
        hoodLookupTable.put(4.65, 21d);
        hoodLookupTable.put(4.70, 23d);
        hoodLookupTable.put(4.75, 23d);
        hoodLookupTable.put(4.80, 22d);
        hoodLookupTable.put(4.85, 22d);
        hoodLookupTable.put(4.90, 21d);
        hoodLookupTable.put(4.95, 23d);
        hoodLookupTable.put(5.00, 23d);
        hoodLookupTable.put(5.05, 22d);
        hoodLookupTable.put(5.10, 22d);
        hoodLookupTable.put(5.15, 23d);
        hoodLookupTable.put(5.20, 24d);
        hoodLookupTable.put(5.25, 23d);
        hoodLookupTable.put(5.30, 22d);
        hoodLookupTable.put(5.35, 24d);
        hoodLookupTable.put(5.40, 23d);
        hoodLookupTable.put(5.45, 24d);
        hoodLookupTable.put(5.50, 23d);
        hoodLookupTable.put(5.55, 22d);
        hoodLookupTable.put(5.60, 24d);
        hoodLookupTable.put(5.65, 24d);
        hoodLookupTable.put(5.70, 23d);
        hoodLookupTable.put(5.75, 23d);
        hoodLookupTable.put(5.80, 24d);
        hoodLookupTable.put(5.85, 24d);
        hoodLookupTable.put(5.90, 24d);
        hoodLookupTable.put(5.95, 23d);
        hoodLookupTable.put(6.00, 24d);
        hoodLookupTable.put(6.05, 25d);
        hoodLookupTable.put(6.10, 24d);
        hoodLookupTable.put(6.15, 23d);
        hoodLookupTable.put(6.20, 24d);
        hoodLookupTable.put(6.25, 25d);
        hoodLookupTable.put(6.30, 24d);
        hoodLookupTable.put(6.35, 23d);
        hoodLookupTable.put(6.40, 24d);
        hoodLookupTable.put(6.45, 25d);
        hoodLookupTable.put(6.50, 24d);
        hoodLookupTable.put(6.55, 25d);
        hoodLookupTable.put(6.60, 24d);
        hoodLookupTable.put(6.65, 25d);
        hoodLookupTable.put(6.70, 24d);
        hoodLookupTable.put(6.75, 25d);
        hoodLookupTable.put(6.80, 24d);
        hoodLookupTable.put(6.85, 25d);
        hoodLookupTable.put(6.90, 24d);
        hoodLookupTable.put(6.95, 25d);
        hoodLookupTable.put(7.00, 24d);
        hoodLookupTable.put(7.05, 25d);
        hoodLookupTable.put(7.10, 24d);
        hoodLookupTable.put(7.15, 25d);
        hoodLookupTable.put(7.20, 25d);
        hoodLookupTable.put(7.25, 25d);
        hoodLookupTable.put(7.30, 25d);
        hoodLookupTable.put(7.35, 25d);
        hoodLookupTable.put(7.40, 25d);
        hoodLookupTable.put(7.45, 25d);
        hoodLookupTable.put(7.50, 25d);
        hoodLookupTable.put(7.55, 25d);
        hoodLookupTable.put(7.60, 25d);
        hoodLookupTable.put(7.65, 24d);
        hoodLookupTable.put(7.70, 25d);
        hoodLookupTable.put(7.75, 25d);
        hoodLookupTable.put(7.80, 25d);
        hoodLookupTable.put(7.85, 25d);
        hoodLookupTable.put(7.90, 25d);
        hoodLookupTable.put(7.95, 25d);
        hoodLookupTable.put(8.00, 25d);

        shooterSpeedLookupTable.put(1.00, 38.5);
        shooterSpeedLookupTable.put(1.05, 38.5);
        shooterSpeedLookupTable.put(1.10, 38.8);
        shooterSpeedLookupTable.put(1.15, 39.1);
        shooterSpeedLookupTable.put(1.20, 39.4);
        shooterSpeedLookupTable.put(1.25, 39.4);
        shooterSpeedLookupTable.put(1.30, 39.6);
        shooterSpeedLookupTable.put(1.35, 39.6);
        shooterSpeedLookupTable.put(1.40, 40.2);
        shooterSpeedLookupTable.put(1.45, 40.2);
        shooterSpeedLookupTable.put(1.50, 40.5);
        shooterSpeedLookupTable.put(1.55, 41d);
        shooterSpeedLookupTable.put(1.60, 41.1);
        shooterSpeedLookupTable.put(1.65, 41.4);
        shooterSpeedLookupTable.put(1.70, 41.7);
        shooterSpeedLookupTable.put(1.75, 41.7);
        shooterSpeedLookupTable.put(1.80, 42d);
        shooterSpeedLookupTable.put(1.85, 42.1);
        shooterSpeedLookupTable.put(1.90, 42.6);
        shooterSpeedLookupTable.put(1.95, 42.9);
        shooterSpeedLookupTable.put(2.00, 42.9);
        shooterSpeedLookupTable.put(2.05, 42.6);
        shooterSpeedLookupTable.put(2.10, 42.6);
        shooterSpeedLookupTable.put(2.15, 42.9);
        shooterSpeedLookupTable.put(2.20, 43.2);
        shooterSpeedLookupTable.put(2.25, 43.5);
        shooterSpeedLookupTable.put(2.30, 43.8);
        shooterSpeedLookupTable.put(2.35, 43.8);
        shooterSpeedLookupTable.put(2.40, 44d);
        shooterSpeedLookupTable.put(2.45, 44.3);
        shooterSpeedLookupTable.put(2.50, 44.6);
        shooterSpeedLookupTable.put(2.55, 44.9);
        shooterSpeedLookupTable.put(2.60, 45.2);
        shooterSpeedLookupTable.put(2.65, 45.5);
        shooterSpeedLookupTable.put(2.70, 45.5);
        shooterSpeedLookupTable.put(2.75, 45.8);
        shooterSpeedLookupTable.put(2.80, 46.1);
        shooterSpeedLookupTable.put(2.85, 46.4);
        shooterSpeedLookupTable.put(2.90, 46.7);
        shooterSpeedLookupTable.put(2.95, 47d);
        shooterSpeedLookupTable.put(3.00, 47.3);
        shooterSpeedLookupTable.put(3.05, 47.3);
        shooterSpeedLookupTable.put(3.10, 47.6);
        shooterSpeedLookupTable.put(3.15, 47.9);
        shooterSpeedLookupTable.put(3.20, 48.1);
        shooterSpeedLookupTable.put(3.25, 48.4);
        shooterSpeedLookupTable.put(3.30, 48.7);
        shooterSpeedLookupTable.put(3.35, 48.9);
        shooterSpeedLookupTable.put(3.40, 49d);
        shooterSpeedLookupTable.put(3.45, 49.3);
        shooterSpeedLookupTable.put(3.50, 49.6);
        shooterSpeedLookupTable.put(3.55, 49.9);
        shooterSpeedLookupTable.put(3.60, 50.2);
        shooterSpeedLookupTable.put(3.65, 50.3);
        shooterSpeedLookupTable.put(3.70, 50.5);
        shooterSpeedLookupTable.put(3.75, 50.8);
        shooterSpeedLookupTable.put(3.80, 51.1);
        shooterSpeedLookupTable.put(3.85, 51.4);
        shooterSpeedLookupTable.put(3.90, 51.7);
        shooterSpeedLookupTable.put(3.95, 51.7);
        shooterSpeedLookupTable.put(4.00, 52d);
        shooterSpeedLookupTable.put(4.05, 52.2);
        shooterSpeedLookupTable.put(4.10, 52.5);
        shooterSpeedLookupTable.put(4.15, 52.8);
        shooterSpeedLookupTable.put(4.20, 53d);
        shooterSpeedLookupTable.put(4.25, 53.1);
        shooterSpeedLookupTable.put(4.30, 53.4);
        shooterSpeedLookupTable.put(4.35, 53.7);
        shooterSpeedLookupTable.put(4.40, 54d);
        shooterSpeedLookupTable.put(4.45, 54.2);
        shooterSpeedLookupTable.put(4.50, 54.3);
        shooterSpeedLookupTable.put(4.55, 54.6);
        shooterSpeedLookupTable.put(4.60, 54.9);
        shooterSpeedLookupTable.put(4.65, 55.2);
        shooterSpeedLookupTable.put(4.70, 55.3);
        shooterSpeedLookupTable.put(4.75, 55.5);
        shooterSpeedLookupTable.put(4.80, 55.8);
        shooterSpeedLookupTable.put(4.85, 56.1);
        shooterSpeedLookupTable.put(4.90, 56.3);
        shooterSpeedLookupTable.put(4.95, 56.5);
        shooterSpeedLookupTable.put(5.00, 56.6);
        shooterSpeedLookupTable.put(5.05, 56.9);
        shooterSpeedLookupTable.put(5.10, 57.2);
        shooterSpeedLookupTable.put(5.15, 57.4);
        shooterSpeedLookupTable.put(5.20, 57.5);
        shooterSpeedLookupTable.put(5.25, 57.8);
        shooterSpeedLookupTable.put(5.30, 58.1);
        shooterSpeedLookupTable.put(5.35, 58.3);
        shooterSpeedLookupTable.put(5.40, 58.5);
        shooterSpeedLookupTable.put(5.45, 58.7);
        shooterSpeedLookupTable.put(5.50, 59d);
        shooterSpeedLookupTable.put(5.55, 59.3);
        shooterSpeedLookupTable.put(5.60, 59.4);
        shooterSpeedLookupTable.put(5.65, 59.6);
        shooterSpeedLookupTable.put(5.70, 59.9);
        shooterSpeedLookupTable.put(5.75, 60.2);
        shooterSpeedLookupTable.put(5.80, 60.3);
        shooterSpeedLookupTable.put(5.85, 60.4);
        shooterSpeedLookupTable.put(5.90, 60.7);
        shooterSpeedLookupTable.put(5.95, 61d);
        shooterSpeedLookupTable.put(6.00, 61.2);
        shooterSpeedLookupTable.put(6.05, 61.3);
        shooterSpeedLookupTable.put(6.10, 61.6);
        shooterSpeedLookupTable.put(6.15, 61.9);
        shooterSpeedLookupTable.put(6.20, 62.1);
        shooterSpeedLookupTable.put(6.25, 62.2);
        shooterSpeedLookupTable.put(6.30, 62.5);
        shooterSpeedLookupTable.put(6.35, 62.8);
        shooterSpeedLookupTable.put(6.40, 62.9);
        shooterSpeedLookupTable.put(6.45, 63.1);
        shooterSpeedLookupTable.put(6.50, 63.4);
        shooterSpeedLookupTable.put(6.55, 63.5);
        shooterSpeedLookupTable.put(6.60, 63.8);
        shooterSpeedLookupTable.put(6.65, 64d);
        shooterSpeedLookupTable.put(6.70, 64.3);
        shooterSpeedLookupTable.put(6.75, 64.4);
        shooterSpeedLookupTable.put(6.80, 64.7);
        shooterSpeedLookupTable.put(6.85, 64.8);
        shooterSpeedLookupTable.put(6.90, 65.1);
        shooterSpeedLookupTable.put(6.95, 65.3);
        shooterSpeedLookupTable.put(7.00, 65.6);
        shooterSpeedLookupTable.put(7.05, 65.7);
        shooterSpeedLookupTable.put(7.10, 66d);
        shooterSpeedLookupTable.put(7.15, 66.2);
        shooterSpeedLookupTable.put(7.20, 66.3);
        shooterSpeedLookupTable.put(7.25, 66.6);
        shooterSpeedLookupTable.put(7.30, 66.7);
        shooterSpeedLookupTable.put(7.35, 67d);
        shooterSpeedLookupTable.put(7.40, 67.2);
        shooterSpeedLookupTable.put(7.45, 67.5);
        shooterSpeedLookupTable.put(7.50, 67.6);
        shooterSpeedLookupTable.put(7.55, 67.9);
        shooterSpeedLookupTable.put(7.60, 68.1);
        shooterSpeedLookupTable.put(7.65, 68.4);
        shooterSpeedLookupTable.put(7.70, 68.5);
        shooterSpeedLookupTable.put(7.75, 68.7);
        shooterSpeedLookupTable.put(7.80, 68.9);
        shooterSpeedLookupTable.put(7.85, 69.1);
        shooterSpeedLookupTable.put(7.90, 69.4);
        shooterSpeedLookupTable.put(7.95, 69.5);
        shooterSpeedLookupTable.put(8.00, 69.8);

        timeOfFlightLookupTable.put(1.00, 0.711);
        timeOfFlightLookupTable.put(1.05, 0.701);
        timeOfFlightLookupTable.put(1.10, 0.729);
        timeOfFlightLookupTable.put(1.15, 0.713);
        timeOfFlightLookupTable.put(1.20, 0.74);
        timeOfFlightLookupTable.put(1.25, 0.729);
        timeOfFlightLookupTable.put(1.30, 0.753);
        timeOfFlightLookupTable.put(1.35, 0.742);
        timeOfFlightLookupTable.put(1.40, 0.759);
        timeOfFlightLookupTable.put(1.45, 0.749);
        timeOfFlightLookupTable.put(1.50, 0.77);
        timeOfFlightLookupTable.put(1.55, 0.787);
        timeOfFlightLookupTable.put(1.60, 0.773);
        timeOfFlightLookupTable.put(1.65, 0.792);
        timeOfFlightLookupTable.put(1.70, 0.811);
        timeOfFlightLookupTable.put(1.75, 0.799);
        timeOfFlightLookupTable.put(1.80, 0.817);
        timeOfFlightLookupTable.put(1.85, 0.802);
        timeOfFlightLookupTable.put(1.90, 0.816);
        timeOfFlightLookupTable.put(1.95, 0.832);
        timeOfFlightLookupTable.put(2.00, 0.82);
        timeOfFlightLookupTable.put(2.05, 0.785);
        timeOfFlightLookupTable.put(2.10, 0.75);
        timeOfFlightLookupTable.put(2.15, 0.763);
        timeOfFlightLookupTable.put(2.20, 0.776);
        timeOfFlightLookupTable.put(2.25, 0.789);
        timeOfFlightLookupTable.put(2.30, 0.802);
        timeOfFlightLookupTable.put(2.35, 0.746);
        timeOfFlightLookupTable.put(2.40, 0.757);
        timeOfFlightLookupTable.put(2.45, 0.792);
        timeOfFlightLookupTable.put(2.50, 0.804);
        timeOfFlightLookupTable.put(2.55, 0.815);
        timeOfFlightLookupTable.put(2.60, 0.826);
        timeOfFlightLookupTable.put(2.65, 0.838);
        timeOfFlightLookupTable.put(2.70, 0.782);
        timeOfFlightLookupTable.put(2.75, 0.815);
        timeOfFlightLookupTable.put(2.80, 0.825);
        timeOfFlightLookupTable.put(2.85, 0.835);
        timeOfFlightLookupTable.put(2.90, 0.845);
        timeOfFlightLookupTable.put(2.95, 0.881);
        timeOfFlightLookupTable.put(3.00, 0.891);
        timeOfFlightLookupTable.put(3.05, 0.833);
        timeOfFlightLookupTable.put(3.10, 0.865);
        timeOfFlightLookupTable.put(3.15, 0.874);
        timeOfFlightLookupTable.put(3.20, 0.883);
        timeOfFlightLookupTable.put(3.25, 0.892);
        timeOfFlightLookupTable.put(3.30, 0.927);
        timeOfFlightLookupTable.put(3.35, 0.866);
        timeOfFlightLookupTable.put(3.40, 0.877);
        timeOfFlightLookupTable.put(3.45, 0.908);
        timeOfFlightLookupTable.put(3.50, 0.917);
        timeOfFlightLookupTable.put(3.55, 0.925);
        timeOfFlightLookupTable.put(3.60, 0.959);
        timeOfFlightLookupTable.put(3.65, 0.897);
        timeOfFlightLookupTable.put(3.70, 0.907);
        timeOfFlightLookupTable.put(3.75, 0.938);
        timeOfFlightLookupTable.put(3.80, 0.946);
        timeOfFlightLookupTable.put(3.85, 0.979);
        timeOfFlightLookupTable.put(3.90, 0.987);
        timeOfFlightLookupTable.put(3.95, 0.927);
        timeOfFlightLookupTable.put(4.00, 0.958);
        timeOfFlightLookupTable.put(4.05, 0.965);
        timeOfFlightLookupTable.put(4.10, 0.997);
        timeOfFlightLookupTable.put(4.15, 1.004);
        timeOfFlightLookupTable.put(4.20, 0.965);
        timeOfFlightLookupTable.put(4.25, 0.975);
        timeOfFlightLookupTable.put(4.30, 1.006);
        timeOfFlightLookupTable.put(4.35, 1.013);
        timeOfFlightLookupTable.put(4.40, 1.046);
        timeOfFlightLookupTable.put(4.45, 1.004);
        timeOfFlightLookupTable.put(4.50, 1.014);
        timeOfFlightLookupTable.put(4.55, 1.02);
        timeOfFlightLookupTable.put(4.60, 1.052);
        timeOfFlightLookupTable.put(4.65, 1.059);
        timeOfFlightLookupTable.put(4.70, 1.018);
        timeOfFlightLookupTable.put(4.75, 1.027);
        timeOfFlightLookupTable.put(4.80, 1.057);
        timeOfFlightLookupTable.put(4.85, 1.064);
        timeOfFlightLookupTable.put(4.90, 1.097);
        timeOfFlightLookupTable.put(4.95, 1.054);
        timeOfFlightLookupTable.put(5.00, 1.062);
        timeOfFlightLookupTable.put(5.05, 1.094);
        timeOfFlightLookupTable.put(5.10, 1.1);
        timeOfFlightLookupTable.put(5.15, 1.083);
        timeOfFlightLookupTable.put(5.20, 1.067);
        timeOfFlightLookupTable.put(5.25, 1.097);
        timeOfFlightLookupTable.put(5.30, 1.129);
        timeOfFlightLookupTable.put(5.35, 1.086);
        timeOfFlightLookupTable.put(5.40, 1.117);
        timeOfFlightLookupTable.put(5.45, 1.1);
        timeOfFlightLookupTable.put(5.50, 1.131);
        timeOfFlightLookupTable.put(5.55, 1.163);
        timeOfFlightLookupTable.put(5.60, 1.118);
        timeOfFlightLookupTable.put(5.65, 1.126);
        timeOfFlightLookupTable.put(5.70, 1.158);
        timeOfFlightLookupTable.put(5.75, 1.163);
        timeOfFlightLookupTable.put(5.80, 1.145);
        timeOfFlightLookupTable.put(5.85, 1.153);
        timeOfFlightLookupTable.put(5.90, 1.158);
        timeOfFlightLookupTable.put(5.95, 1.19);
        timeOfFlightLookupTable.put(6.00, 1.171);
        timeOfFlightLookupTable.put(6.05, 1.153);
        timeOfFlightLookupTable.put(6.10, 1.184);
        timeOfFlightLookupTable.put(6.15, 1.216);
        timeOfFlightLookupTable.put(6.20, 1.196);
        timeOfFlightLookupTable.put(6.25, 1.178);
        timeOfFlightLookupTable.put(6.30, 1.209);
        timeOfFlightLookupTable.put(6.35, 1.242);
        timeOfFlightLookupTable.put(6.40, 1.221);
        timeOfFlightLookupTable.put(6.45, 1.202);
        timeOfFlightLookupTable.put(6.50, 1.234);
        timeOfFlightLookupTable.put(6.55, 1.214);
        timeOfFlightLookupTable.put(6.60, 1.246);
        timeOfFlightLookupTable.put(6.65, 1.226);
        timeOfFlightLookupTable.put(6.70, 1.258);
        timeOfFlightLookupTable.put(6.75, 1.238);
        timeOfFlightLookupTable.put(6.80, 1.27);
        timeOfFlightLookupTable.put(6.85, 1.249);
        timeOfFlightLookupTable.put(6.90, 1.282);
        timeOfFlightLookupTable.put(6.95, 1.261);
        timeOfFlightLookupTable.put(7.00, 1.294);
        timeOfFlightLookupTable.put(7.05, 1.272);
        timeOfFlightLookupTable.put(7.10, 1.306);
        timeOfFlightLookupTable.put(7.15, 1.284);
        timeOfFlightLookupTable.put(7.20, 1.291);
        timeOfFlightLookupTable.put(7.25, 1.295);
        timeOfFlightLookupTable.put(7.30, 1.302);
        timeOfFlightLookupTable.put(7.35, 1.306);
        timeOfFlightLookupTable.put(7.40, 1.313);
        timeOfFlightLookupTable.put(7.45, 1.317);
        timeOfFlightLookupTable.put(7.50, 1.324);
        timeOfFlightLookupTable.put(7.55, 1.329);
        timeOfFlightLookupTable.put(7.60, 1.335);
        timeOfFlightLookupTable.put(7.65, 1.37);
        timeOfFlightLookupTable.put(7.70, 1.346);
        timeOfFlightLookupTable.put(7.75, 1.353);
        timeOfFlightLookupTable.put(7.80, 1.357);
        timeOfFlightLookupTable.put(7.85, 1.364);
        timeOfFlightLookupTable.put(7.90, 1.368);
        timeOfFlightLookupTable.put(7.95, 1.375);
        timeOfFlightLookupTable.put(8.00, 1.379);

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

            Translation2d target = getCurrentTarget();
            ChassisSpeeds robotRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
                    RobotContainer.getSpeeds(), RobotContainer.getPose().getRotation());

            Pair<Pose2d, Double> turretPhaseDelayed = createFutureTurretPose(estimatedPosePhaseDelayed, robotRelativeSpeeds, target);
            Pair<Pose2d, Double> turret = createFutureTurretPose(estimatedPose, robotRelativeSpeeds, target);

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

            turretRotationPhaseDelayed = target.minus(futureTurretPositionPhaseDelayed.getTranslation()).getAngle();

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
            turretRotation = target.minus(futureTurretPosition.getTranslation()).getAngle();

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


    public Pair<Pose2d, Double> createFutureTurretPose(Pose2d pose2d, ChassisSpeeds fieldRelativeVelocity, Translation2d target) {
        Pose2d turretPosition = pose2d.transformBy(Constants.ROBOT_TO_TURRET);

        // Sets distance values for lookup tables based on ShotMode
        double baseX = turretPosition.getX();
        double baseY = turretPosition.getY();
        double targetX = target.getX();
        double targetY = target.getY();
        double turretToTargetDistance = Math.hypot(targetX - baseX, targetY - baseY);

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
        double futureX = baseX, futureY = baseY;
        double lookaheadTurretToTargetDistance = turretToTargetDistance;

//        Iterate to converge on a final future position
        for (int i = 0; i < 10; i++) {
            timeOfFlight = timeOfFlightLookupTable.get(lookaheadTurretToTargetDistance);
            futureX = baseX + turretVelocityX * timeOfFlight;
            futureY = baseY + turretVelocityY * timeOfFlight;

            double newDistance = Math.hypot(targetX - futureX, targetY - futureY);
            boolean converged = Math.abs(newDistance - lookaheadTurretToTargetDistance) < CONVERGENCE_EPSILON_METERS;
            lookaheadTurretToTargetDistance = newDistance;
            if (converged) break;
        }

        Pose2d futureTurretPosition = new Pose2d(futureX, futureY, turretPosition.getRotation());
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

    private Translation2d getCurrentTarget() {
        ShotMode mode = RobotContainer.getShotMode();
        if (mode == ShotMode.SHOOTING) {
            return AllianceFlipper.getCorrectAlliance(Constants.BLUE_HUB_CENTER, Constants.RED_HUB_CENTER);
        } else if (mode == ShotMode.SHUTTLING_LEFT) {
            return AllianceFlipper.getCorrectAlliance(Constants.BLUE_SHUTTLE_LEFT_CORNER, Constants.RED_SHUTTLE_LEFT_CORNER);
        } else {
            return AllianceFlipper.getCorrectAlliance(Constants.BLUE_SHUTTLE_RIGHT_CORNER, Constants.RED_SHUTTLE_RIGHT_CORNER);
        }
    }
}

