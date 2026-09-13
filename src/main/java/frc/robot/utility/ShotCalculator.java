package frc.robot.utility;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
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
        
        hoodLookupTable.put(1.10, 0d);
        hoodLookupTable.put(1.15, 0d);
        hoodLookupTable.put(1.20, 0d);
        hoodLookupTable.put(1.25, 1d);
        hoodLookupTable.put(1.30, 1d);
        hoodLookupTable.put(1.35, 1d);
        hoodLookupTable.put(1.40, 2d);
        hoodLookupTable.put(1.45, 2d);
        hoodLookupTable.put(1.50, 3d);
        hoodLookupTable.put(1.55, 3d);
        hoodLookupTable.put(1.60, 3d);
        hoodLookupTable.put(1.65, 3d);
        hoodLookupTable.put(1.70, 4d);
        hoodLookupTable.put(1.75, 4d);
        hoodLookupTable.put(1.80, 4d);
        hoodLookupTable.put(1.85, 5d);
        hoodLookupTable.put(1.90, 5d);
        hoodLookupTable.put(1.95, 5d);
        hoodLookupTable.put(2.00, 5d);
        hoodLookupTable.put(2.05, 6d);
        hoodLookupTable.put(2.10, 6d);
        hoodLookupTable.put(2.15, 6d);
        hoodLookupTable.put(2.20, 6d);
        hoodLookupTable.put(2.25, 6d);
        hoodLookupTable.put(2.30, 7d);
        hoodLookupTable.put(2.35, 7d);
        hoodLookupTable.put(2.40, 7d);
        hoodLookupTable.put(2.45, 7d);
        hoodLookupTable.put(2.50, 7d);
        hoodLookupTable.put(2.55, 8d);
        hoodLookupTable.put(2.60, 8d);
        hoodLookupTable.put(2.65, 8d);
        hoodLookupTable.put(2.70, 8d);
        hoodLookupTable.put(2.75, 8d);
        hoodLookupTable.put(2.80, 9d);
        hoodLookupTable.put(2.85, 9d);
        hoodLookupTable.put(2.90, 9d);
        hoodLookupTable.put(2.95, 9d);
        hoodLookupTable.put(3.00, 9d);
        hoodLookupTable.put(3.05, 9d);
        hoodLookupTable.put(3.10, 9d);
        hoodLookupTable.put(3.15, 10d);
        hoodLookupTable.put(3.20, 10d);
        hoodLookupTable.put(3.25, 10d);
        hoodLookupTable.put(3.30, 10d);
        hoodLookupTable.put(3.35, 10d);
        hoodLookupTable.put(3.40, 10d);
        hoodLookupTable.put(3.45, 10d);
        hoodLookupTable.put(3.50, 11d);
        hoodLookupTable.put(3.55, 11d);
        hoodLookupTable.put(3.60, 11d);
        hoodLookupTable.put(3.65, 11d);
        hoodLookupTable.put(3.70, 11d);
        hoodLookupTable.put(3.75, 11d);
        hoodLookupTable.put(3.80, 11d);
        hoodLookupTable.put(3.85, 11d);
        hoodLookupTable.put(3.90, 12d);
        hoodLookupTable.put(3.95, 12d);
        hoodLookupTable.put(4.00, 12d);
        hoodLookupTable.put(4.05, 12d);
        hoodLookupTable.put(4.10, 12d);
        hoodLookupTable.put(4.15, 12d);
        hoodLookupTable.put(4.20, 12d);
        hoodLookupTable.put(4.25, 12d);
        hoodLookupTable.put(4.30, 12d);
        hoodLookupTable.put(4.35, 13d);
        hoodLookupTable.put(4.40, 13d);
        hoodLookupTable.put(4.45, 13d);
        hoodLookupTable.put(4.50, 13d);
        hoodLookupTable.put(4.55, 13d);
        hoodLookupTable.put(4.60, 13d);
        hoodLookupTable.put(4.65, 13d);
        hoodLookupTable.put(4.70, 13d);
        hoodLookupTable.put(4.75, 13d);
        hoodLookupTable.put(4.80, 14d);
        hoodLookupTable.put(4.85, 14d);
        hoodLookupTable.put(4.90, 14d);
        hoodLookupTable.put(4.95, 14d);
        hoodLookupTable.put(5.00, 14d);
        hoodLookupTable.put(5.05, 14d);
        hoodLookupTable.put(5.10, 14d);
        hoodLookupTable.put(5.15, 14d);
        hoodLookupTable.put(5.20, 14d);
        hoodLookupTable.put(5.25, 14d);
        hoodLookupTable.put(5.30, 14d);
        hoodLookupTable.put(5.35, 14d);
        hoodLookupTable.put(5.40, 14d);
        hoodLookupTable.put(5.45, 15d);
        hoodLookupTable.put(5.50, 15d);
        hoodLookupTable.put(5.55, 15d);
        hoodLookupTable.put(5.60, 15d);
        hoodLookupTable.put(5.65, 15d);
        hoodLookupTable.put(5.70, 15d);
        hoodLookupTable.put(5.75, 15d);
        hoodLookupTable.put(5.80, 15d);
        hoodLookupTable.put(5.85, 15d);
        hoodLookupTable.put(5.90, 15d);
        hoodLookupTable.put(5.95, 15d);
        hoodLookupTable.put(6.00, 15d);
        hoodLookupTable.put(6.05, 15d);
        hoodLookupTable.put(6.10, 15d);
        hoodLookupTable.put(6.15, 16d);
        hoodLookupTable.put(6.20, 16d);
        hoodLookupTable.put(6.25, 16d);
        hoodLookupTable.put(6.30, 16d);
        hoodLookupTable.put(6.35, 16d);
        hoodLookupTable.put(6.40, 16d);
        hoodLookupTable.put(6.45, 16d);
        hoodLookupTable.put(6.50, 16d);
        hoodLookupTable.put(6.55, 16d);
        hoodLookupTable.put(6.60, 16d);
        hoodLookupTable.put(6.65, 16d);
        hoodLookupTable.put(6.70, 16d);
        hoodLookupTable.put(6.75, 16d);
        hoodLookupTable.put(6.80, 16d);
        hoodLookupTable.put(6.85, 16d);
        hoodLookupTable.put(6.90, 17d);
        hoodLookupTable.put(6.95, 17d);
        hoodLookupTable.put(7.00, 17d);
        hoodLookupTable.put(7.05, 17d);
        hoodLookupTable.put(7.10, 17d);
        hoodLookupTable.put(7.15, 17d);
        hoodLookupTable.put(7.20, 17d);
        hoodLookupTable.put(7.25, 17d);
        hoodLookupTable.put(7.30, 17d);
        hoodLookupTable.put(7.35, 17d);
        hoodLookupTable.put(7.40, 17d);
        hoodLookupTable.put(7.45, 17d);
        hoodLookupTable.put(7.50, 17d);
        hoodLookupTable.put(7.55, 17d);
        hoodLookupTable.put(7.60, 17d);
        hoodLookupTable.put(7.65, 17d);
        hoodLookupTable.put(7.70, 18d);
        hoodLookupTable.put(7.75, 18d);
        hoodLookupTable.put(7.80, 18d);
        hoodLookupTable.put(7.85, 18d);
        hoodLookupTable.put(7.90, 18d);
        hoodLookupTable.put(7.95, 18d);
        hoodLookupTable.put(8.00, 18d);

        shooterSpeedLookupTable.put(1.10, 41.7);
        shooterSpeedLookupTable.put(1.15, 42.3);
        shooterSpeedLookupTable.put(1.20, 42.9);
        shooterSpeedLookupTable.put(1.25, 42.6);
        shooterSpeedLookupTable.put(1.30, 43.2);
        shooterSpeedLookupTable.put(1.35, 43.8);
        shooterSpeedLookupTable.put(1.40, 43.5);
        shooterSpeedLookupTable.put(1.45, 44d);
        shooterSpeedLookupTable.put(1.50, 43.8);
        shooterSpeedLookupTable.put(1.55, 44.3);
        shooterSpeedLookupTable.put(1.60, 44.9);
        shooterSpeedLookupTable.put(1.65, 45.2);
        shooterSpeedLookupTable.put(1.70, 45.2);
        shooterSpeedLookupTable.put(1.75, 45.5);
        shooterSpeedLookupTable.put(1.80, 46.1);
        shooterSpeedLookupTable.put(1.85, 45.8);
        shooterSpeedLookupTable.put(1.90, 46.4);
        shooterSpeedLookupTable.put(1.95, 46.7);
        shooterSpeedLookupTable.put(2.00, 47.3);
        shooterSpeedLookupTable.put(2.05, 47d);
        shooterSpeedLookupTable.put(2.10, 47.4);
        shooterSpeedLookupTable.put(2.15, 47.9);
        shooterSpeedLookupTable.put(2.20, 48.1);
        shooterSpeedLookupTable.put(2.25, 48.7);
        shooterSpeedLookupTable.put(2.30, 48.4);
        shooterSpeedLookupTable.put(2.35, 48.9);
        shooterSpeedLookupTable.put(2.40, 49.3);
        shooterSpeedLookupTable.put(2.45, 49.6);
        shooterSpeedLookupTable.put(2.50, 50d);
        shooterSpeedLookupTable.put(2.55, 49.9);
        shooterSpeedLookupTable.put(2.60, 50.2);
        shooterSpeedLookupTable.put(2.65, 50.6);
        shooterSpeedLookupTable.put(2.70, 51.1);
        shooterSpeedLookupTable.put(2.75, 51.4);
        shooterSpeedLookupTable.put(2.80, 51.1);
        shooterSpeedLookupTable.put(2.85, 51.5);
        shooterSpeedLookupTable.put(2.90, 52d);
        shooterSpeedLookupTable.put(2.95, 52.2);
        shooterSpeedLookupTable.put(3.00, 52.5);
        shooterSpeedLookupTable.put(3.05, 53d);
        shooterSpeedLookupTable.put(3.10, 53.3);
        shooterSpeedLookupTable.put(3.15, 53.1);
        shooterSpeedLookupTable.put(3.20, 53.4);
        shooterSpeedLookupTable.put(3.25, 53.7);
        shooterSpeedLookupTable.put(3.30, 54.2);
        shooterSpeedLookupTable.put(3.35, 54.4);
        shooterSpeedLookupTable.put(3.40, 54.9);
        shooterSpeedLookupTable.put(3.45, 55.2);
        shooterSpeedLookupTable.put(3.50, 54.9);
        shooterSpeedLookupTable.put(3.55, 55.2);
        shooterSpeedLookupTable.put(3.60, 55.6);
        shooterSpeedLookupTable.put(3.65, 55.9);
        shooterSpeedLookupTable.put(3.70, 56.2);
        shooterSpeedLookupTable.put(3.75, 56.6);
        shooterSpeedLookupTable.put(3.80, 56.9);
        shooterSpeedLookupTable.put(3.85, 57.2);
        shooterSpeedLookupTable.put(3.90, 56.9);
        shooterSpeedLookupTable.put(3.95, 57.2);
        shooterSpeedLookupTable.put(4.00, 57.7);
        shooterSpeedLookupTable.put(4.05, 58d);
        shooterSpeedLookupTable.put(4.10, 58.3);
        shooterSpeedLookupTable.put(4.15, 58.5);
        shooterSpeedLookupTable.put(4.20, 58.8);
        shooterSpeedLookupTable.put(4.25, 59.1);
        shooterSpeedLookupTable.put(4.30, 59.6);
        shooterSpeedLookupTable.put(4.35, 59.3);
        shooterSpeedLookupTable.put(4.40, 59.6);
        shooterSpeedLookupTable.put(4.45, 59.9);
        shooterSpeedLookupTable.put(4.50, 60.2);
        shooterSpeedLookupTable.put(4.55, 60.4);
        shooterSpeedLookupTable.put(4.60, 60.7);
        shooterSpeedLookupTable.put(4.65, 61d);
        shooterSpeedLookupTable.put(4.70, 61.3);
        shooterSpeedLookupTable.put(4.75, 61.6);
        shooterSpeedLookupTable.put(4.80, 61.3);
        shooterSpeedLookupTable.put(4.85, 61.6);
        shooterSpeedLookupTable.put(4.90, 61.9);
        shooterSpeedLookupTable.put(4.95, 62.2);
        shooterSpeedLookupTable.put(5.00, 62.5);
        shooterSpeedLookupTable.put(5.05, 62.8);
        shooterSpeedLookupTable.put(5.10, 63.1);
        shooterSpeedLookupTable.put(5.15, 63.4);
        shooterSpeedLookupTable.put(5.20, 63.7);
        shooterSpeedLookupTable.put(5.25, 64d);
        shooterSpeedLookupTable.put(5.30, 64.3);
        shooterSpeedLookupTable.put(5.35, 64.6);
        shooterSpeedLookupTable.put(5.40, 64.8);
        shooterSpeedLookupTable.put(5.45, 64.6);
        shooterSpeedLookupTable.put(5.50, 64.8);
        shooterSpeedLookupTable.put(5.55, 65.1);
        shooterSpeedLookupTable.put(5.60, 65.4);
        shooterSpeedLookupTable.put(5.65, 65.7);
        shooterSpeedLookupTable.put(5.70, 66d);
        shooterSpeedLookupTable.put(5.75, 66.2);
        shooterSpeedLookupTable.put(5.80, 66.5);
        shooterSpeedLookupTable.put(5.85, 66.7);
        shooterSpeedLookupTable.put(5.90, 67d);
        shooterSpeedLookupTable.put(5.95, 67.3);
        shooterSpeedLookupTable.put(6.00, 67.6);
        shooterSpeedLookupTable.put(6.05, 67.9);
        shooterSpeedLookupTable.put(6.10, 68.1);
        shooterSpeedLookupTable.put(6.15, 67.8);
        shooterSpeedLookupTable.put(6.20, 68.1);
        shooterSpeedLookupTable.put(6.25, 68.4);
        shooterSpeedLookupTable.put(6.30, 68.7);
        shooterSpeedLookupTable.put(6.35, 68.9);
        shooterSpeedLookupTable.put(6.40, 69.2);
        shooterSpeedLookupTable.put(6.45, 69.4);
        shooterSpeedLookupTable.put(6.50, 69.7);
        shooterSpeedLookupTable.put(6.55, 70d);
        shooterSpeedLookupTable.put(6.60, 70.3);
        shooterSpeedLookupTable.put(6.65, 70.5);
        shooterSpeedLookupTable.put(6.70, 70.7);
        shooterSpeedLookupTable.put(6.75, 71d);
        shooterSpeedLookupTable.put(6.80, 71.3);
        shooterSpeedLookupTable.put(6.85, 71.6);
        shooterSpeedLookupTable.put(6.90, 71.1);
        shooterSpeedLookupTable.put(6.95, 71.4);
        shooterSpeedLookupTable.put(7.00, 71.7);
        shooterSpeedLookupTable.put(7.05, 72d);
        shooterSpeedLookupTable.put(7.10, 72.2);
        shooterSpeedLookupTable.put(7.15, 72.5);
        shooterSpeedLookupTable.put(7.20, 72.8);
        shooterSpeedLookupTable.put(7.25, 73d);
        shooterSpeedLookupTable.put(7.30, 73.2);
        shooterSpeedLookupTable.put(7.35, 73.5);
        shooterSpeedLookupTable.put(7.40, 73.8);
        shooterSpeedLookupTable.put(7.45, 74d);
        shooterSpeedLookupTable.put(7.50, 74.2);
        shooterSpeedLookupTable.put(7.55, 74.5);
        shooterSpeedLookupTable.put(7.60, 74.8);
        shooterSpeedLookupTable.put(7.65, 75d);
        shooterSpeedLookupTable.put(7.70, 74.7);
        shooterSpeedLookupTable.put(7.75, 75d);
        shooterSpeedLookupTable.put(7.80, 75.2);
        shooterSpeedLookupTable.put(7.85, 75.4);
        shooterSpeedLookupTable.put(7.90, 75.7);
        shooterSpeedLookupTable.put(7.95, 75.9);
        shooterSpeedLookupTable.put(8.00, 76.1);

        timeOfFlightLookupTable.put(1.10, 0.764);
        timeOfFlightLookupTable.put(1.15, 0.789);
        timeOfFlightLookupTable.put(1.20, 0.813);
        timeOfFlightLookupTable.put(1.25, 0.801);
        timeOfFlightLookupTable.put(1.30, 0.822);
        timeOfFlightLookupTable.put(1.35, 0.843);
        timeOfFlightLookupTable.put(1.40, 0.83);
        timeOfFlightLookupTable.put(1.45, 0.849);
        timeOfFlightLookupTable.put(1.50, 0.836);
        timeOfFlightLookupTable.put(1.55, 0.854);
        timeOfFlightLookupTable.put(1.60, 0.871);
        timeOfFlightLookupTable.put(1.65, 0.893);
        timeOfFlightLookupTable.put(1.70, 0.873);
        timeOfFlightLookupTable.put(1.75, 0.894);
        timeOfFlightLookupTable.put(1.80, 0.909);
        timeOfFlightLookupTable.put(1.85, 0.894);
        timeOfFlightLookupTable.put(1.90, 0.908);
        timeOfFlightLookupTable.put(1.95, 0.927);
        timeOfFlightLookupTable.put(2.00, 0.94);
        timeOfFlightLookupTable.put(2.05, 0.925);
        timeOfFlightLookupTable.put(2.10, 0.94);
        timeOfFlightLookupTable.put(2.15, 0.954);
        timeOfFlightLookupTable.put(2.20, 0.971);
        timeOfFlightLookupTable.put(2.25, 0.983);
        timeOfFlightLookupTable.put(2.30, 0.966);
        timeOfFlightLookupTable.put(2.35, 0.979);
        timeOfFlightLookupTable.put(2.40, 0.992);
        timeOfFlightLookupTable.put(2.45, 1.008);
        timeOfFlightLookupTable.put(2.50, 1.02);
        timeOfFlightLookupTable.put(2.55, 1.0);
        timeOfFlightLookupTable.put(2.60, 1.015);
        timeOfFlightLookupTable.put(2.65, 1.027);
        timeOfFlightLookupTable.put(2.70, 1.038);
        timeOfFlightLookupTable.put(2.75, 1.052);
        timeOfFlightLookupTable.put(2.80, 1.034);
        timeOfFlightLookupTable.put(2.85, 1.045);
        timeOfFlightLookupTable.put(2.90, 1.055);
        timeOfFlightLookupTable.put(2.95, 1.068);
        timeOfFlightLookupTable.put(3.00, 1.082);
        timeOfFlightLookupTable.put(3.05, 1.091);
        timeOfFlightLookupTable.put(3.10, 1.104);
        timeOfFlightLookupTable.put(3.15, 1.082);
        timeOfFlightLookupTable.put(3.20, 1.094);
        timeOfFlightLookupTable.put(3.25, 1.107);
        timeOfFlightLookupTable.put(3.30, 1.115);
        timeOfFlightLookupTable.put(3.35, 1.127);
        timeOfFlightLookupTable.put(3.40, 1.136);
        timeOfFlightLookupTable.put(3.45, 1.148);
        timeOfFlightLookupTable.put(3.50, 1.128);
        timeOfFlightLookupTable.put(3.55, 1.139);
        timeOfFlightLookupTable.put(3.60, 1.147);
        timeOfFlightLookupTable.put(3.65, 1.158);
        timeOfFlightLookupTable.put(3.70, 1.169);
        timeOfFlightLookupTable.put(3.75, 1.176);
        timeOfFlightLookupTable.put(3.80, 1.187);
        timeOfFlightLookupTable.put(3.85, 1.197);
        timeOfFlightLookupTable.put(3.90, 1.176);
        timeOfFlightLookupTable.put(3.95, 1.186);
        timeOfFlightLookupTable.put(4.00, 1.193);
        timeOfFlightLookupTable.put(4.05, 1.203);
        timeOfFlightLookupTable.put(4.10, 1.213);
        timeOfFlightLookupTable.put(4.15, 1.223);
        timeOfFlightLookupTable.put(4.20, 1.233);
        timeOfFlightLookupTable.put(4.25, 1.242);
        timeOfFlightLookupTable.put(4.30, 1.249);
        timeOfFlightLookupTable.put(4.35, 1.227);
        timeOfFlightLookupTable.put(4.40, 1.236);
        timeOfFlightLookupTable.put(4.45, 1.245);
        timeOfFlightLookupTable.put(4.50, 1.254);
        timeOfFlightLookupTable.put(4.55, 1.263);
        timeOfFlightLookupTable.put(4.60, 1.271);
        timeOfFlightLookupTable.put(4.65, 1.28);
        timeOfFlightLookupTable.put(4.70, 1.289);
        timeOfFlightLookupTable.put(4.75, 1.298);
        timeOfFlightLookupTable.put(4.80, 1.275);
        timeOfFlightLookupTable.put(4.85, 1.283);
        timeOfFlightLookupTable.put(4.90, 1.291);
        timeOfFlightLookupTable.put(4.95, 1.3);
        timeOfFlightLookupTable.put(5.00, 1.308);
        timeOfFlightLookupTable.put(5.05, 1.316);
        timeOfFlightLookupTable.put(5.10, 1.324);
        timeOfFlightLookupTable.put(5.15, 1.332);
        timeOfFlightLookupTable.put(5.20, 1.34);
        timeOfFlightLookupTable.put(5.25, 1.348);
        timeOfFlightLookupTable.put(5.30, 1.355);
        timeOfFlightLookupTable.put(5.35, 1.363);
        timeOfFlightLookupTable.put(5.40, 1.371);
        timeOfFlightLookupTable.put(5.45, 1.346);
        timeOfFlightLookupTable.put(5.50, 1.354);
        timeOfFlightLookupTable.put(5.55, 1.361);
        timeOfFlightLookupTable.put(5.60, 1.368);
        timeOfFlightLookupTable.put(5.65, 1.376);
        timeOfFlightLookupTable.put(5.70, 1.383);
        timeOfFlightLookupTable.put(5.75, 1.393);
        timeOfFlightLookupTable.put(5.80, 1.4);
        timeOfFlightLookupTable.put(5.85, 1.407);
        timeOfFlightLookupTable.put(5.90, 1.414);
        timeOfFlightLookupTable.put(5.95, 1.421);
        timeOfFlightLookupTable.put(6.00, 1.428);
        timeOfFlightLookupTable.put(6.05, 1.435);
        timeOfFlightLookupTable.put(6.10, 1.444);
        timeOfFlightLookupTable.put(6.15, 1.419);
        timeOfFlightLookupTable.put(6.20, 1.426);
        timeOfFlightLookupTable.put(6.25, 1.432);
        timeOfFlightLookupTable.put(6.30, 1.439);
        timeOfFlightLookupTable.put(6.35, 1.445);
        timeOfFlightLookupTable.put(6.40, 1.453);
        timeOfFlightLookupTable.put(6.45, 1.461);
        timeOfFlightLookupTable.put(6.50, 1.468);
        timeOfFlightLookupTable.put(6.55, 1.474);
        timeOfFlightLookupTable.put(6.60, 1.48);
        timeOfFlightLookupTable.put(6.65, 1.488);
        timeOfFlightLookupTable.put(6.70, 1.496);
        timeOfFlightLookupTable.put(6.75, 1.502);
        timeOfFlightLookupTable.put(6.80, 1.509);
        timeOfFlightLookupTable.put(6.85, 1.515);
        timeOfFlightLookupTable.put(6.90, 1.49);
        timeOfFlightLookupTable.put(6.95, 1.496);
        timeOfFlightLookupTable.put(7.00, 1.502);
        timeOfFlightLookupTable.put(7.05, 1.508);
        timeOfFlightLookupTable.put(7.10, 1.517);
        timeOfFlightLookupTable.put(7.15, 1.523);
        timeOfFlightLookupTable.put(7.20, 1.528);
        timeOfFlightLookupTable.put(7.25, 1.534);
        timeOfFlightLookupTable.put(7.30, 1.543);
        timeOfFlightLookupTable.put(7.35, 1.549);
        timeOfFlightLookupTable.put(7.40, 1.554);
        timeOfFlightLookupTable.put(7.45, 1.562);
        timeOfFlightLookupTable.put(7.50, 1.569);
        timeOfFlightLookupTable.put(7.55, 1.574);
        timeOfFlightLookupTable.put(7.60, 1.58);
        timeOfFlightLookupTable.put(7.65, 1.589);
        timeOfFlightLookupTable.put(7.70, 1.56);
        timeOfFlightLookupTable.put(7.75, 1.565);
        timeOfFlightLookupTable.put(7.80, 1.572);
        timeOfFlightLookupTable.put(7.85, 1.579);
        timeOfFlightLookupTable.put(7.90, 1.584);
        timeOfFlightLookupTable.put(7.95, 1.591);
        timeOfFlightLookupTable.put(8.00, 1.597);
        //----------------

        shuttleHoodLookupTable.put(2.00, 25d);
        shuttleHoodLookupTable.put(2.50, 25d);
        shuttleHoodLookupTable.put(3.00, 25d);
        shuttleHoodLookupTable.put(3.50, 25d);
        shuttleHoodLookupTable.put(4.00, 25d);
        shuttleHoodLookupTable.put(4.50, 25d);
        shuttleHoodLookupTable.put(5.00, 25d);
        shuttleHoodLookupTable.put(5.50, 25d);
        shuttleHoodLookupTable.put(6.00, 25d);
        shuttleHoodLookupTable.put(6.50, 25d);
        shuttleHoodLookupTable.put(7.00, 25d);
        shuttleHoodLookupTable.put(7.50, 25d);
        shuttleHoodLookupTable.put(8.00, 25d);
        shuttleHoodLookupTable.put(8.50, 25d);
        shuttleHoodLookupTable.put(9.00, 25d);
        shuttleHoodLookupTable.put(9.50, 25d);
        shuttleHoodLookupTable.put(10.00, 25d);
        shuttleHoodLookupTable.put(10.50, 25d);
        shuttleHoodLookupTable.put(11.00, 25d);
        shuttleHoodLookupTable.put(11.50, 25d);
        shuttleHoodLookupTable.put(12.00, 25d);
        shuttleHoodLookupTable.put(12.50, 25d);

        shuttleShooterSpeedLookupTable.put(2.00, 29.9);
        shuttleShooterSpeedLookupTable.put(2.50, 34.2);
        shuttleShooterSpeedLookupTable.put(3.00, 38d);
        shuttleShooterSpeedLookupTable.put(3.50, 41.6);
        shuttleShooterSpeedLookupTable.put(4.00, 44.9);
        shuttleShooterSpeedLookupTable.put(4.50, 48d);
        shuttleShooterSpeedLookupTable.put(5.00, 51.1);
        shuttleShooterSpeedLookupTable.put(5.50, 53.9);
        shuttleShooterSpeedLookupTable.put(6.00, 56.7);
        shuttleShooterSpeedLookupTable.put(6.50, 59.4);
        shuttleShooterSpeedLookupTable.put(7.00, 62d);
        shuttleShooterSpeedLookupTable.put(7.50, 64.6);
        shuttleShooterSpeedLookupTable.put(8.00, 67d);
        shuttleShooterSpeedLookupTable.put(8.50, 69.5);
        shuttleShooterSpeedLookupTable.put(9.00, 71.8);
        shuttleShooterSpeedLookupTable.put(9.50, 74.1);
        shuttleShooterSpeedLookupTable.put(10.00, 76.5);
        shuttleShooterSpeedLookupTable.put(10.50, 78.8);
        shuttleShooterSpeedLookupTable.put(11.00, 81d);
        shuttleShooterSpeedLookupTable.put(11.50, 83.2);
        shuttleShooterSpeedLookupTable.put(12.00, 85.4);
        shuttleShooterSpeedLookupTable.put(12.50, 85.4);

        shuttleTimeOfFlightLookupTable.put(2.00, 0.778);
        shuttleTimeOfFlightLookupTable.put(2.50, 0.858);
        shuttleTimeOfFlightLookupTable.put(3.00, 0.931);
        shuttleTimeOfFlightLookupTable.put(3.50, 1.002);
        shuttleTimeOfFlightLookupTable.put(4.00, 1.067);
        shuttleTimeOfFlightLookupTable.put(4.50, 1.131);
        shuttleTimeOfFlightLookupTable.put(5.00, 1.19);
        shuttleTimeOfFlightLookupTable.put(5.50, 1.249);
        shuttleTimeOfFlightLookupTable.put(6.00, 1.306);
        shuttleTimeOfFlightLookupTable.put(6.50, 1.362);
        shuttleTimeOfFlightLookupTable.put(7.00, 1.415);
        shuttleTimeOfFlightLookupTable.put(7.50, 1.467);
        shuttleTimeOfFlightLookupTable.put(8.00, 1.519);
        shuttleTimeOfFlightLookupTable.put(8.50, 1.568);
        shuttleTimeOfFlightLookupTable.put(9.00, 1.619);
        shuttleTimeOfFlightLookupTable.put(9.50, 1.668);
        shuttleTimeOfFlightLookupTable.put(10.00, 1.715);
        shuttleTimeOfFlightLookupTable.put(10.50, 1.762);
        shuttleTimeOfFlightLookupTable.put(11.00, 1.81);
        shuttleTimeOfFlightLookupTable.put(11.50, 1.855);
        shuttleTimeOfFlightLookupTable.put(12.00, 1.901);
        shuttleTimeOfFlightLookupTable.put(12.50, 1.901);

//        shuttleHoodLookupTable.put(4.726098, 10d + 2d + 2d);
//        shuttleHoodLookupTable.put(7.415803, 16d + 2d + 2d);
//        shuttleHoodLookupTable.put(10.540514, 24d);
//        shuttleHoodLookupTable.put(10.970932, 24d);
//
//        shuttleShooterSpeedLookupTable.put(4.726098, 50d - 5d);
//        shuttleShooterSpeedLookupTable.put(7.415803, 79d - 5d);
//        shuttleShooterSpeedLookupTable.put(10.540514, 100d);
//        shuttleShooterSpeedLookupTable.put(10.970932, 100d);
//        shuttleShooterSpeedLookupTable.put(12d, 100d);
//
//        shuttleTimeOfFlightLookupTable.put(4.726098, 1.37 + .075);
//        shuttleTimeOfFlightLookupTable.put(7.415803, 1.46 + .075);
//        shuttleTimeOfFlightLookupTable.put(10.540514, 1.55 + .075);
//        shuttleTimeOfFlightLookupTable.put(10.970932, 1.56 + .075);

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


        //Offset for left corner
        if (futureTurretToTargetDistance >= 4) {
            if ((Robot.getAlliance().equals(DriverStation.Alliance.Red)
                    && futureTurretPosition.getY() < Constants.RED_HUB_CENTER.getY())
                    || (Robot.getAlliance().equals(DriverStation.Alliance.Blue)
                    && futureTurretPosition.getY() > Constants.BLUE_HUB_CENTER.getY())) {
                turretAngle += 1d;
                turretAnglePhaseDelayed += 1d;
            }
        }
        if (futureTurretToTargetDistance >= 4.5) {
            if ((Robot.getAlliance().equals(DriverStation.Alliance.Red)
                    && futureTurretPosition.getY() < Constants.RED_HUB_CENTER.getY())
                    || (Robot.getAlliance().equals(DriverStation.Alliance.Blue)
                    && futureTurretPosition.getY() > Constants.BLUE_HUB_CENTER.getY())) {
                turretAngle += .75;
                turretAnglePhaseDelayed += .75;
            }
        }

        turretSubsystem.updateGoalPosition(getTurretAnglePhaseDelayed(), getTurretVelocityPhaseDelayed());
        RobotContainer.getShooterControlAuto().setGoal(shotCalculator.getShooterSpeedPhaseDelayed());
        RobotContainer.getShooterControlAutoRB().setGoal(shotCalculator.getShooterSpeedPhaseDelayed());
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

