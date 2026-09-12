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
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

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
        hoodLookupTable.put(1.70, 7d);
        hoodLookupTable.put(1.75, 7d);
        hoodLookupTable.put(1.80, 7d);
        hoodLookupTable.put(1.85, 8d);
        hoodLookupTable.put(1.90, 8d);
        hoodLookupTable.put(1.95, 8d);
        hoodLookupTable.put(2.00, 8d);
        hoodLookupTable.put(2.05, 9d);
        hoodLookupTable.put(2.10, 9d);
        hoodLookupTable.put(2.15, 10d);
        hoodLookupTable.put(2.20, 10d);
        hoodLookupTable.put(2.25, 10d);
        hoodLookupTable.put(2.30, 10d);
        hoodLookupTable.put(2.35, 11d);
        hoodLookupTable.put(2.40, 11d);
        hoodLookupTable.put(2.45, 11d);
        hoodLookupTable.put(2.50, 12d);
        hoodLookupTable.put(2.55, 12d);
        hoodLookupTable.put(2.60, 12d);
        hoodLookupTable.put(2.65, 12d);
        hoodLookupTable.put(2.70, 12d);
        hoodLookupTable.put(2.75, 13d);
        hoodLookupTable.put(2.80, 13d);
        hoodLookupTable.put(2.85, 13d);
        hoodLookupTable.put(2.90, 13d);
        hoodLookupTable.put(2.95, 14d);
        hoodLookupTable.put(3.00, 14d);
        hoodLookupTable.put(3.05, 14d);
        hoodLookupTable.put(3.10, 14d);
        hoodLookupTable.put(3.15, 14d);
        hoodLookupTable.put(3.20, 15d);
        hoodLookupTable.put(3.25, 15d);
        hoodLookupTable.put(3.30, 15d);
        hoodLookupTable.put(3.35, 15d);
        hoodLookupTable.put(3.40, 15d);
        hoodLookupTable.put(3.45, 16d);
        hoodLookupTable.put(3.50, 16d);
        hoodLookupTable.put(3.55, 16d);
        hoodLookupTable.put(3.60, 16d);
        hoodLookupTable.put(3.65, 16d);
        hoodLookupTable.put(3.70, 16d);
        hoodLookupTable.put(3.75, 16d);
        hoodLookupTable.put(3.80, 17d);
        hoodLookupTable.put(3.85, 17d);
        hoodLookupTable.put(3.90, 17d);
        hoodLookupTable.put(3.95, 17d);
        hoodLookupTable.put(4.00, 17d);
        hoodLookupTable.put(4.05, 17d);
        hoodLookupTable.put(4.10, 18d);
        hoodLookupTable.put(4.15, 18d);
        hoodLookupTable.put(4.20, 18d);
        hoodLookupTable.put(4.25, 18d);
        hoodLookupTable.put(4.30, 18d);
        hoodLookupTable.put(4.35, 18d);
        hoodLookupTable.put(4.40, 19d);
        hoodLookupTable.put(4.45, 19d);
        hoodLookupTable.put(4.50, 19d);
        hoodLookupTable.put(4.55, 19d);
        hoodLookupTable.put(4.60, 19d);
        hoodLookupTable.put(4.65, 19d);
        hoodLookupTable.put(4.70, 19d);
        hoodLookupTable.put(4.75, 19d);
        hoodLookupTable.put(4.80, 19d);
        hoodLookupTable.put(4.85, 19d);
        hoodLookupTable.put(4.90, 20d);
        hoodLookupTable.put(4.95, 20d);
        hoodLookupTable.put(5.00, 20d);
        hoodLookupTable.put(5.05, 20d);
        hoodLookupTable.put(5.10, 20d);
        hoodLookupTable.put(5.15, 20d);
        hoodLookupTable.put(5.20, 20d);
        hoodLookupTable.put(5.25, 20d);
        hoodLookupTable.put(5.30, 21d);
        hoodLookupTable.put(5.35, 21d);
        hoodLookupTable.put(5.40, 21d);
        hoodLookupTable.put(5.45, 21d);
        hoodLookupTable.put(5.50, 21d);
        hoodLookupTable.put(5.55, 21d);
        hoodLookupTable.put(5.60, 21d);
        hoodLookupTable.put(5.65, 21d);
        hoodLookupTable.put(5.70, 21d);
        hoodLookupTable.put(5.75, 21d);
        hoodLookupTable.put(5.80, 22d);
        hoodLookupTable.put(5.85, 22d);
        hoodLookupTable.put(5.90, 22d);
        hoodLookupTable.put(5.95, 22d);
        hoodLookupTable.put(6.00, 22d);
        hoodLookupTable.put(6.05, 22d);
        hoodLookupTable.put(6.10, 22d);
        hoodLookupTable.put(6.15, 22d);
        hoodLookupTable.put(6.20, 22d);
        hoodLookupTable.put(6.25, 22d);
        hoodLookupTable.put(6.30, 23d);
        hoodLookupTable.put(6.35, 23d);
        hoodLookupTable.put(6.40, 23d);
        hoodLookupTable.put(6.45, 23d);
        hoodLookupTable.put(6.50, 23d);
        hoodLookupTable.put(6.55, 23d);
        hoodLookupTable.put(6.60, 23d);
        hoodLookupTable.put(6.65, 23d);
        hoodLookupTable.put(6.70, 23d);
        hoodLookupTable.put(6.75, 23d);
        hoodLookupTable.put(6.80, 23d);
        hoodLookupTable.put(6.85, 23d);
        hoodLookupTable.put(6.90, 24d);
        hoodLookupTable.put(6.95, 24d);
        hoodLookupTable.put(7.00, 24d);
        hoodLookupTable.put(7.05, 24d);
        hoodLookupTable.put(7.10, 24d);
        hoodLookupTable.put(7.15, 24d);
        hoodLookupTable.put(7.20, 24d);
        hoodLookupTable.put(7.25, 24d);
        hoodLookupTable.put(7.30, 24d);
        hoodLookupTable.put(7.35, 24d);
        hoodLookupTable.put(7.40, 24d);
        hoodLookupTable.put(7.45, 24d);
        hoodLookupTable.put(7.50, 25d);
        hoodLookupTable.put(7.55, 25d);
        hoodLookupTable.put(7.60, 25d);
        hoodLookupTable.put(7.65, 25d);
        hoodLookupTable.put(7.70, 25d);
        hoodLookupTable.put(7.75, 25d);
        hoodLookupTable.put(7.80, 25d);
        hoodLookupTable.put(7.85, 25d);
        hoodLookupTable.put(7.90, 25d);
        hoodLookupTable.put(7.95, 25d);
        hoodLookupTable.put(8.00, 25d);

        shooterSpeedLookupTable.put(1.00, 40.2);
        shooterSpeedLookupTable.put(1.05, 40.2);
        shooterSpeedLookupTable.put(1.10, 40.8);
        shooterSpeedLookupTable.put(1.15, 40.8);
        shooterSpeedLookupTable.put(1.20, 41.1);
        shooterSpeedLookupTable.put(1.25, 41.1);
        shooterSpeedLookupTable.put(1.30, 41.7);
        shooterSpeedLookupTable.put(1.35, 41.7);
        shooterSpeedLookupTable.put(1.40, 42d);
        shooterSpeedLookupTable.put(1.45, 42d);
        shooterSpeedLookupTable.put(1.50, 42.6);
        shooterSpeedLookupTable.put(1.55, 42.9);
        shooterSpeedLookupTable.put(1.60, 42.9);
        shooterSpeedLookupTable.put(1.65, 43.2);
        shooterSpeedLookupTable.put(1.70, 43.2);
        shooterSpeedLookupTable.put(1.75, 43.8);
        shooterSpeedLookupTable.put(1.80, 44d);
        shooterSpeedLookupTable.put(1.85, 44d);
        shooterSpeedLookupTable.put(1.90, 44.3);
        shooterSpeedLookupTable.put(1.95, 44.8);
        shooterSpeedLookupTable.put(2.00, 45.2);
        shooterSpeedLookupTable.put(2.05, 45.2);
        shooterSpeedLookupTable.put(2.10, 45.5);
        shooterSpeedLookupTable.put(2.15, 45.5);
        shooterSpeedLookupTable.put(2.20, 45.8);
        shooterSpeedLookupTable.put(2.25, 46.2);
        shooterSpeedLookupTable.put(2.30, 46.7);
        shooterSpeedLookupTable.put(2.35, 46.7);
        shooterSpeedLookupTable.put(2.40, 47d);
        shooterSpeedLookupTable.put(2.45, 47.3);
        shooterSpeedLookupTable.put(2.50, 47.3);
        shooterSpeedLookupTable.put(2.55, 47.6);
        shooterSpeedLookupTable.put(2.60, 47.9);
        shooterSpeedLookupTable.put(2.65, 48.3);
        shooterSpeedLookupTable.put(2.70, 48.7);
        shooterSpeedLookupTable.put(2.75, 48.7);
        shooterSpeedLookupTable.put(2.80, 49d);
        shooterSpeedLookupTable.put(2.85, 49.3);
        shooterSpeedLookupTable.put(2.90, 49.6);
        shooterSpeedLookupTable.put(2.95, 49.6);
        shooterSpeedLookupTable.put(3.00, 49.9);
        shooterSpeedLookupTable.put(3.05, 50.2);
        shooterSpeedLookupTable.put(3.10, 50.5);
        shooterSpeedLookupTable.put(3.15, 50.8);
        shooterSpeedLookupTable.put(3.20, 50.8);
        shooterSpeedLookupTable.put(3.25, 51.1);
        shooterSpeedLookupTable.put(3.30, 51.4);
        shooterSpeedLookupTable.put(3.35, 51.8);
        shooterSpeedLookupTable.put(3.40, 52.1);
        shooterSpeedLookupTable.put(3.45, 52.1);
        shooterSpeedLookupTable.put(3.50, 52.4);
        shooterSpeedLookupTable.put(3.55, 52.7);
        shooterSpeedLookupTable.put(3.60, 53d);
        shooterSpeedLookupTable.put(3.65, 53.3);
        shooterSpeedLookupTable.put(3.70, 53.6);
        shooterSpeedLookupTable.put(3.75, 53.9);
        shooterSpeedLookupTable.put(3.80, 53.9);
        shooterSpeedLookupTable.put(3.85, 54.2);
        shooterSpeedLookupTable.put(3.90, 54.4);
        shooterSpeedLookupTable.put(3.95, 54.7);
        shooterSpeedLookupTable.put(4.00, 55d);
        shooterSpeedLookupTable.put(4.05, 55.2);
        shooterSpeedLookupTable.put(4.10, 55.3);
        shooterSpeedLookupTable.put(4.15, 55.6);
        shooterSpeedLookupTable.put(4.20, 55.8);
        shooterSpeedLookupTable.put(4.25, 56.1);
        shooterSpeedLookupTable.put(4.30, 56.3);
        shooterSpeedLookupTable.put(4.35, 56.6);
        shooterSpeedLookupTable.put(4.40, 56.6);
        shooterSpeedLookupTable.put(4.45, 56.9);
        shooterSpeedLookupTable.put(4.50, 57.2);
        shooterSpeedLookupTable.put(4.55, 57.5);
        shooterSpeedLookupTable.put(4.60, 57.8);
        shooterSpeedLookupTable.put(4.65, 58.1);
        shooterSpeedLookupTable.put(4.70, 58.4);
        shooterSpeedLookupTable.put(4.75, 58.5);
        shooterSpeedLookupTable.put(4.80, 58.8);
        shooterSpeedLookupTable.put(4.85, 59.1);
        shooterSpeedLookupTable.put(4.90, 59.1);
        shooterSpeedLookupTable.put(4.95, 59.4);
        shooterSpeedLookupTable.put(5.00, 59.6);
        shooterSpeedLookupTable.put(5.05, 59.9);
        shooterSpeedLookupTable.put(5.10, 60.2);
        shooterSpeedLookupTable.put(5.15, 60.4);
        shooterSpeedLookupTable.put(5.20, 60.7);
        shooterSpeedLookupTable.put(5.25, 60.9);
        shooterSpeedLookupTable.put(5.30, 61d);
        shooterSpeedLookupTable.put(5.35, 61.3);
        shooterSpeedLookupTable.put(5.40, 61.5);
        shooterSpeedLookupTable.put(5.45, 61.8);
        shooterSpeedLookupTable.put(5.50, 61.9);
        shooterSpeedLookupTable.put(5.55, 62.2);
        shooterSpeedLookupTable.put(5.60, 62.5);
        shooterSpeedLookupTable.put(5.65, 62.8);
        shooterSpeedLookupTable.put(5.70, 62.9);
        shooterSpeedLookupTable.put(5.75, 63.2);
        shooterSpeedLookupTable.put(5.80, 63.2);
        shooterSpeedLookupTable.put(5.85, 63.5);
        shooterSpeedLookupTable.put(5.90, 63.7);
        shooterSpeedLookupTable.put(5.95, 64d);
        shooterSpeedLookupTable.put(6.00, 64.3);
        shooterSpeedLookupTable.put(6.05, 64.6);
        shooterSpeedLookupTable.put(6.10, 64.7);
        shooterSpeedLookupTable.put(6.15, 65d);
        shooterSpeedLookupTable.put(6.20, 65.1);
        shooterSpeedLookupTable.put(6.25, 65.4);
        shooterSpeedLookupTable.put(6.30, 65.4);
        shooterSpeedLookupTable.put(6.35, 65.7);
        shooterSpeedLookupTable.put(6.40, 66d);
        shooterSpeedLookupTable.put(6.45, 66.2);
        shooterSpeedLookupTable.put(6.50, 66.5);
        shooterSpeedLookupTable.put(6.55, 66.6);
        shooterSpeedLookupTable.put(6.60, 66.9);
        shooterSpeedLookupTable.put(6.65, 67.2);
        shooterSpeedLookupTable.put(6.70, 67.3);
        shooterSpeedLookupTable.put(6.75, 67.6);
        shooterSpeedLookupTable.put(6.80, 67.8);
        shooterSpeedLookupTable.put(6.85, 68.1);
        shooterSpeedLookupTable.put(6.90, 68.1);
        shooterSpeedLookupTable.put(6.95, 68.4);
        shooterSpeedLookupTable.put(7.00, 68.7);
        shooterSpeedLookupTable.put(7.05, 68.8);
        shooterSpeedLookupTable.put(7.10, 69.1);
        shooterSpeedLookupTable.put(7.15, 69.2);
        shooterSpeedLookupTable.put(7.20, 69.5);
        shooterSpeedLookupTable.put(7.25, 69.8);
        shooterSpeedLookupTable.put(7.30, 70d);
        shooterSpeedLookupTable.put(7.35, 70.1);
        shooterSpeedLookupTable.put(7.40, 70.4);
        shooterSpeedLookupTable.put(7.45, 70.7);
        shooterSpeedLookupTable.put(7.50, 70.7);
        shooterSpeedLookupTable.put(7.55, 71d);
        shooterSpeedLookupTable.put(7.60, 71.1);
        shooterSpeedLookupTable.put(7.65, 71.4);
        shooterSpeedLookupTable.put(7.70, 71.6);
        shooterSpeedLookupTable.put(7.75, 71.9);
        shooterSpeedLookupTable.put(7.80, 72d);
        shooterSpeedLookupTable.put(7.85, 72.3);
        shooterSpeedLookupTable.put(7.90, 72.5);
        shooterSpeedLookupTable.put(7.95, 72.8);
        shooterSpeedLookupTable.put(8.00, 72.9);

        timeOfFlightLookupTable.put(1.00, 0.71);
        timeOfFlightLookupTable.put(1.05, 0.7);
        timeOfFlightLookupTable.put(1.10, 0.724);
        timeOfFlightLookupTable.put(1.15, 0.714);
        timeOfFlightLookupTable.put(1.20, 0.74);
        timeOfFlightLookupTable.put(1.25, 0.729);
        timeOfFlightLookupTable.put(1.30, 0.749);
        timeOfFlightLookupTable.put(1.35, 0.738);
        timeOfFlightLookupTable.put(1.40, 0.761);
        timeOfFlightLookupTable.put(1.45, 0.75);
        timeOfFlightLookupTable.put(1.50, 0.766);
        timeOfFlightLookupTable.put(1.55, 0.787);
        timeOfFlightLookupTable.put(1.60, 0.775);
        timeOfFlightLookupTable.put(1.65, 0.795);
        timeOfFlightLookupTable.put(1.70, 0.783);
        timeOfFlightLookupTable.put(1.75, 0.796);
        timeOfFlightLookupTable.put(1.80, 0.814);
        timeOfFlightLookupTable.put(1.85, 0.802);
        timeOfFlightLookupTable.put(1.90, 0.819);
        timeOfFlightLookupTable.put(1.95, 0.833);
        timeOfFlightLookupTable.put(2.00, 0.847);
        timeOfFlightLookupTable.put(2.05, 0.834);
        timeOfFlightLookupTable.put(2.10, 0.85);
        timeOfFlightLookupTable.put(2.15, 0.837);
        timeOfFlightLookupTable.put(2.20, 0.852);
        timeOfFlightLookupTable.put(2.25, 0.864);
        timeOfFlightLookupTable.put(2.30, 0.875);
        timeOfFlightLookupTable.put(2.35, 0.862);
        timeOfFlightLookupTable.put(2.40, 0.876);
        timeOfFlightLookupTable.put(2.45, 0.889);
        timeOfFlightLookupTable.put(2.50, 0.876);
        timeOfFlightLookupTable.put(2.55, 0.889);
        timeOfFlightLookupTable.put(2.60, 0.902);
        timeOfFlightLookupTable.put(2.65, 0.911);
        timeOfFlightLookupTable.put(2.70, 0.921);
        timeOfFlightLookupTable.put(2.75, 0.907);
        timeOfFlightLookupTable.put(2.80, 0.919);
        timeOfFlightLookupTable.put(2.85, 0.93);
        timeOfFlightLookupTable.put(2.90, 0.942);
        timeOfFlightLookupTable.put(2.95, 0.928);
        timeOfFlightLookupTable.put(3.00, 0.939);
        timeOfFlightLookupTable.put(3.05, 0.949);
        timeOfFlightLookupTable.put(3.10, 0.96);
        timeOfFlightLookupTable.put(3.15, 0.971);
        timeOfFlightLookupTable.put(3.20, 0.956);
        timeOfFlightLookupTable.put(3.25, 0.966);
        timeOfFlightLookupTable.put(3.30, 0.976);
        timeOfFlightLookupTable.put(3.35, 0.984);
        timeOfFlightLookupTable.put(3.40, 0.994);
        timeOfFlightLookupTable.put(3.45, 0.979);
        timeOfFlightLookupTable.put(3.50, 0.988);
        timeOfFlightLookupTable.put(3.55, 0.997);
        timeOfFlightLookupTable.put(3.60, 1.007);
        timeOfFlightLookupTable.put(3.65, 1.016);
        timeOfFlightLookupTable.put(3.70, 1.025);
        timeOfFlightLookupTable.put(3.75, 1.034);
        timeOfFlightLookupTable.put(3.80, 1.018);
        timeOfFlightLookupTable.put(3.85, 1.027);
        timeOfFlightLookupTable.put(3.90, 1.035);
        timeOfFlightLookupTable.put(3.95, 1.044);
        timeOfFlightLookupTable.put(4.00, 1.052);
        timeOfFlightLookupTable.put(4.05, 1.063);
        timeOfFlightLookupTable.put(4.10, 1.044);
        timeOfFlightLookupTable.put(4.15, 1.052);
        timeOfFlightLookupTable.put(4.20, 1.063);
        timeOfFlightLookupTable.put(4.25, 1.071);
        timeOfFlightLookupTable.put(4.30, 1.079);
        timeOfFlightLookupTable.put(4.35, 1.087);
        timeOfFlightLookupTable.put(4.40, 1.07);
        timeOfFlightLookupTable.put(4.45, 1.078);
        timeOfFlightLookupTable.put(4.50, 1.085);
        timeOfFlightLookupTable.put(4.55, 1.092);
        timeOfFlightLookupTable.put(4.60, 1.1);
        timeOfFlightLookupTable.put(4.65, 1.107);
        timeOfFlightLookupTable.put(4.70, 1.114);
        timeOfFlightLookupTable.put(4.75, 1.124);
        timeOfFlightLookupTable.put(4.80, 1.131);
        timeOfFlightLookupTable.put(4.85, 1.138);
        timeOfFlightLookupTable.put(4.90, 1.121);
        timeOfFlightLookupTable.put(4.95, 1.127);
        timeOfFlightLookupTable.put(5.00, 1.137);
        timeOfFlightLookupTable.put(5.05, 1.144);
        timeOfFlightLookupTable.put(5.10, 1.15);
        timeOfFlightLookupTable.put(5.15, 1.157);
        timeOfFlightLookupTable.put(5.20, 1.163);
        timeOfFlightLookupTable.put(5.25, 1.172);
        timeOfFlightLookupTable.put(5.30, 1.152);
        timeOfFlightLookupTable.put(5.35, 1.158);
        timeOfFlightLookupTable.put(5.40, 1.167);
        timeOfFlightLookupTable.put(5.45, 1.173);
        timeOfFlightLookupTable.put(5.50, 1.182);
        timeOfFlightLookupTable.put(5.55, 1.188);
        timeOfFlightLookupTable.put(5.60, 1.194);
        timeOfFlightLookupTable.put(5.65, 1.2);
        timeOfFlightLookupTable.put(5.70, 1.209);
        timeOfFlightLookupTable.put(5.75, 1.215);
        timeOfFlightLookupTable.put(5.80, 1.196);
        timeOfFlightLookupTable.put(5.85, 1.202);
        timeOfFlightLookupTable.put(5.90, 1.21);
        timeOfFlightLookupTable.put(5.95, 1.216);
        timeOfFlightLookupTable.put(6.00, 1.221);
        timeOfFlightLookupTable.put(6.05, 1.227);
        timeOfFlightLookupTable.put(6.10, 1.235);
        timeOfFlightLookupTable.put(6.15, 1.241);
        timeOfFlightLookupTable.put(6.20, 1.249);
        timeOfFlightLookupTable.put(6.25, 1.254);
        timeOfFlightLookupTable.put(6.30, 1.235);
        timeOfFlightLookupTable.put(6.35, 1.241);
        timeOfFlightLookupTable.put(6.40, 1.246);
        timeOfFlightLookupTable.put(6.45, 1.254);
        timeOfFlightLookupTable.put(6.50, 1.259);
        timeOfFlightLookupTable.put(6.55, 1.267);
        timeOfFlightLookupTable.put(6.60, 1.272);
        timeOfFlightLookupTable.put(6.65, 1.277);
        timeOfFlightLookupTable.put(6.70, 1.285);
        timeOfFlightLookupTable.put(6.75, 1.29);
        timeOfFlightLookupTable.put(6.80, 1.297);
        timeOfFlightLookupTable.put(6.85, 1.302);
        timeOfFlightLookupTable.put(6.90, 1.283);
        timeOfFlightLookupTable.put(6.95, 1.287);
        timeOfFlightLookupTable.put(7.00, 1.292);
        timeOfFlightLookupTable.put(7.05, 1.3);
        timeOfFlightLookupTable.put(7.10, 1.304);
        timeOfFlightLookupTable.put(7.15, 1.312);
        timeOfFlightLookupTable.put(7.20, 1.316);
        timeOfFlightLookupTable.put(7.25, 1.321);
        timeOfFlightLookupTable.put(7.30, 1.328);
        timeOfFlightLookupTable.put(7.35, 1.335);
        timeOfFlightLookupTable.put(7.40, 1.34);
        timeOfFlightLookupTable.put(7.45, 1.344);
        timeOfFlightLookupTable.put(7.50, 1.324);
        timeOfFlightLookupTable.put(7.55, 1.329);
        timeOfFlightLookupTable.put(7.60, 1.336);
        timeOfFlightLookupTable.put(7.65, 1.34);
        timeOfFlightLookupTable.put(7.70, 1.347);
        timeOfFlightLookupTable.put(7.75, 1.351);
        timeOfFlightLookupTable.put(7.80, 1.358);
        timeOfFlightLookupTable.put(7.85, 1.363);
        timeOfFlightLookupTable.put(7.90, 1.369);
        timeOfFlightLookupTable.put(7.95, 1.374);
        timeOfFlightLookupTable.put(8.00, 1.381);

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


        // Sets continuous control for turret, shooter, and hood
//        RobotContainer.getTurretControlAuto().setGoal(shotCalculator.getTurretAnglePhaseDelayed(),
//                shotCalculator.getTurretVelocityPhaseDelayed());
        turretSubsystem.updateGoalPosition(getTurretAnglePhaseDelayed(), getTurretVelocityPhaseDelayed());
        RobotContainer.getShooterControlAuto().setGoal(shotCalculator.getShooterSpeedPhaseDelayed());
        RobotContainer.getShooterControlAutoRB().setGoal(shotCalculator.getShooterSpeedPhaseDelayed());
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

    public double getShooterSpeedPhaseDelayed() {
        return shooterSpeedPhaseDelayed;
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

