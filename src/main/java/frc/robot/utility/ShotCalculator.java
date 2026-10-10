package frc.robot.utility;

import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
import frc.robot.constants.TurretConstants;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import org.wpilib.driverstation.Alliance;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Twist2d;
import org.wpilib.math.interpolation.InterpolatingDoubleTreeMap;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.util.Pair;

import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
import frc.robot.constants.TurretConstants;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class ShotCalculator {
    private static final double CONVERGENCE_EPSILON_METERS = 0.005;
    private static ShotCalculator shotCalculator;
    private Pose2d futureTurretPosition = new Pose2d(0, 0, new Rotation2d(0)), futureTurretPositionPhaseDelayed = new Pose2d(0, 0, new Rotation2d(0));
    private double turretAngle, turretAnglePhaseDelayed;
    //    private double lastTurretAngle, lastTurretAnglePhaseDelayed;
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
    //    private ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();
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

        hoodLookupTable.put(1.00, 0.0d);
        hoodLookupTable.put(1.05, 0.3d);
        hoodLookupTable.put(1.10, 0.7d);
        hoodLookupTable.put(1.15, 1.1d);
        hoodLookupTable.put(1.20, 1.4d);
        hoodLookupTable.put(1.25, 1.8d);
        hoodLookupTable.put(1.30, 2.7d);
        hoodLookupTable.put(1.35, 3.0d);
        hoodLookupTable.put(1.40, 3.3d);
        hoodLookupTable.put(1.45, 3.6d);
        hoodLookupTable.put(1.50, 3.9d);
        hoodLookupTable.put(1.55, 4.2d);
        hoodLookupTable.put(1.60, 4.5d);
        hoodLookupTable.put(1.65, 5.4d);
        hoodLookupTable.put(1.70, 5.7d);
        hoodLookupTable.put(1.75, 5.9d);
        hoodLookupTable.put(1.80, 6.2d);
        hoodLookupTable.put(1.85, 6.4d);
        hoodLookupTable.put(1.90, 6.6d);
        hoodLookupTable.put(1.95, 6.9d);
        hoodLookupTable.put(2.00, 7.1d);
        hoodLookupTable.put(2.05, 7.3d);
        hoodLookupTable.put(2.10, 8.2d);
        hoodLookupTable.put(2.15, 8.3d);
        hoodLookupTable.put(2.20, 8.5d);
        hoodLookupTable.put(2.25, 9.4d);
        hoodLookupTable.put(2.30, 9.6d);
        hoodLookupTable.put(2.35, 9.8d);
        hoodLookupTable.put(2.40, 10.7d);
        hoodLookupTable.put(2.45, 10.9d);
        hoodLookupTable.put(2.50, 11.0d);
        hoodLookupTable.put(2.55, 11.1d);
        hoodLookupTable.put(2.60, 12.0d);
        hoodLookupTable.put(2.65, 12.3d);
        hoodLookupTable.put(2.70, 12.4d);
        hoodLookupTable.put(2.75, 12.5d);
        hoodLookupTable.put(2.80, 13.6d);
        hoodLookupTable.put(2.85, 13.7d);
        hoodLookupTable.put(2.90, 13.7d);
        hoodLookupTable.put(2.95, 13.8d);
        hoodLookupTable.put(3.00, 15.0d);
        hoodLookupTable.put(3.05, 15.0d);
        hoodLookupTable.put(3.10, 15.0d);
        hoodLookupTable.put(3.15, 15.0d);
        hoodLookupTable.put(3.20, 16.5d);
        hoodLookupTable.put(3.25, 16.5d);
        hoodLookupTable.put(3.30, 16.5d);
        hoodLookupTable.put(3.35, 16.5d);
        hoodLookupTable.put(3.40, 17.8d);
        hoodLookupTable.put(3.45, 17.9d);
        hoodLookupTable.put(3.50, 17.9d);
        hoodLookupTable.put(3.55, 17.9d);
        hoodLookupTable.put(3.60, 17.9d);
        hoodLookupTable.put(3.65, 19.8d);
        hoodLookupTable.put(3.70, 19.8d);
        hoodLookupTable.put(3.75, 19.8d);
        hoodLookupTable.put(3.80, 19.8d);
        hoodLookupTable.put(3.85, 19.8d);
        hoodLookupTable.put(3.90, 21.3d);
        hoodLookupTable.put(3.95, 21.3d);
        hoodLookupTable.put(4.00, 21.3d);
        hoodLookupTable.put(4.05, 21.3d);
        hoodLookupTable.put(4.10, 21.3d);
        hoodLookupTable.put(4.15, 21.3d);
        hoodLookupTable.put(4.20, 21.3d);
        hoodLookupTable.put(4.25, 22.2d);
        hoodLookupTable.put(4.30, 22.2d);
        hoodLookupTable.put(4.35, 22.2d);
        hoodLookupTable.put(4.40, 22.2d);
        hoodLookupTable.put(4.45, 22.2d);
        hoodLookupTable.put(4.50, 22.4d);
        hoodLookupTable.put(4.55, 22.6d);
        hoodLookupTable.put(4.60, 22.6d);
        hoodLookupTable.put(4.65, 22.6d);
        hoodLookupTable.put(4.70, 22.6d);
        hoodLookupTable.put(4.75, 22.6d);
        hoodLookupTable.put(4.80, 22.6d);
        hoodLookupTable.put(4.85, 22.6d);
        hoodLookupTable.put(4.90, 22.6d);
        hoodLookupTable.put(4.95, 22.6d);
        hoodLookupTable.put(5.00, 22.6d);
        hoodLookupTable.put(5.05, 22.9d);
        hoodLookupTable.put(5.10, 23.3d);
        hoodLookupTable.put(5.15, 23.3d);
        hoodLookupTable.put(5.20, 23.3d);
        hoodLookupTable.put(5.25, 23.3d);
        hoodLookupTable.put(5.30, 23.3d);
        hoodLookupTable.put(5.35, 23.6d);
        hoodLookupTable.put(5.40, 23.6d);
        hoodLookupTable.put(5.45, 23.6d);
        hoodLookupTable.put(5.50, 23.6d);
        hoodLookupTable.put(5.55, 23.6d);
        hoodLookupTable.put(5.60, 23.8d);
        hoodLookupTable.put(5.65, 23.8d);
        hoodLookupTable.put(5.70, 23.8d);
        hoodLookupTable.put(5.75, 24.4d);
        hoodLookupTable.put(5.80, 24.4d);
        hoodLookupTable.put(5.85, 24.4d);
        hoodLookupTable.put(5.90, 24.4d);
        hoodLookupTable.put(5.95, 24.4d);
        hoodLookupTable.put(6.00, 24.4d);
        hoodLookupTable.put(6.05, 24.5d);
        hoodLookupTable.put(6.10, 24.5d);
        hoodLookupTable.put(6.15, 24.5d);
        hoodLookupTable.put(6.20, 24.6d);
        hoodLookupTable.put(6.25, 24.6d);
        hoodLookupTable.put(6.30, 24.6d);
        hoodLookupTable.put(6.35, 24.6d);
        hoodLookupTable.put(6.40, 24.6d);
        hoodLookupTable.put(6.45, 24.6d);
        hoodLookupTable.put(6.50, 24.6d);
        hoodLookupTable.put(6.55, 24.6d);
        hoodLookupTable.put(6.60, 24.6d);
        hoodLookupTable.put(6.65, 24.6d);
        hoodLookupTable.put(6.70, 24.9d);
        hoodLookupTable.put(6.75, 24.9d);
        hoodLookupTable.put(6.80, 24.9d);
        hoodLookupTable.put(6.85, 24.9d);
        hoodLookupTable.put(6.90, 24.9d);
        hoodLookupTable.put(6.95, 24.9d);
        hoodLookupTable.put(7.00, 24.9d);
        hoodLookupTable.put(7.05, 24.9d);
        hoodLookupTable.put(7.10, 24.9d);
        hoodLookupTable.put(7.15, 24.9d);
        hoodLookupTable.put(7.20, 24.9d);
        hoodLookupTable.put(7.25, 24.9d);
        hoodLookupTable.put(7.30, 24.9d);
        hoodLookupTable.put(7.35, 24.9d);
        hoodLookupTable.put(7.40, 24.9d);
        hoodLookupTable.put(7.45, 24.9d);
        hoodLookupTable.put(7.50, 24.9d);
        hoodLookupTable.put(7.55, 24.9d);
        hoodLookupTable.put(7.60, 24.9d);
        hoodLookupTable.put(7.65, 24.9d);
        hoodLookupTable.put(7.70, 24.9d);
        hoodLookupTable.put(7.75, 24.9d);
        hoodLookupTable.put(7.80, 24.9d);
        hoodLookupTable.put(7.85, 24.9d);
        hoodLookupTable.put(7.90, 24.9d);
        hoodLookupTable.put(7.95, 24.9d);
        hoodLookupTable.put(8.00, 24.9d);

        shooterSpeedLookupTable.put(1.00, 39.6);
        shooterSpeedLookupTable.put(1.05, 39.9);
        shooterSpeedLookupTable.put(1.10, 40.2);
        shooterSpeedLookupTable.put(1.15, 40.5);
        shooterSpeedLookupTable.put(1.20, 40.8);
        shooterSpeedLookupTable.put(1.25, 41.1);
        shooterSpeedLookupTable.put(1.30, 41.1);
        shooterSpeedLookupTable.put(1.35, 41.4);
        shooterSpeedLookupTable.put(1.40, 41.7);
        shooterSpeedLookupTable.put(1.45, 42d);
        shooterSpeedLookupTable.put(1.50, 42.3);
        shooterSpeedLookupTable.put(1.55, 42.6);
        shooterSpeedLookupTable.put(1.60, 42.9);
        shooterSpeedLookupTable.put(1.65, 42.9);
        shooterSpeedLookupTable.put(1.70, 43.2);
        shooterSpeedLookupTable.put(1.75, 43.5);
        shooterSpeedLookupTable.put(1.80, 43.8);
        shooterSpeedLookupTable.put(1.85, 44d);
        shooterSpeedLookupTable.put(1.90, 44.3);
        shooterSpeedLookupTable.put(1.95, 44.6);
        shooterSpeedLookupTable.put(2.00, 44.9);
        shooterSpeedLookupTable.put(2.05, 45.2);
        shooterSpeedLookupTable.put(2.10, 45.2);
        shooterSpeedLookupTable.put(2.15, 45.5);
        shooterSpeedLookupTable.put(2.20, 45.8);
        shooterSpeedLookupTable.put(2.25, 45.8);
        shooterSpeedLookupTable.put(2.30, 46.1);
        shooterSpeedLookupTable.put(2.35, 46.4);
        shooterSpeedLookupTable.put(2.40, 46.4);
        shooterSpeedLookupTable.put(2.45, 46.7);
        shooterSpeedLookupTable.put(2.50, 47d);
        shooterSpeedLookupTable.put(2.55, 47.3);
        shooterSpeedLookupTable.put(2.60, 47.4);
        shooterSpeedLookupTable.put(2.65, 47.6);
        shooterSpeedLookupTable.put(2.70, 47.9);
        shooterSpeedLookupTable.put(2.75, 48.1);
        shooterSpeedLookupTable.put(2.80, 48.1);
        shooterSpeedLookupTable.put(2.85, 48.4);
        shooterSpeedLookupTable.put(2.90, 48.7);
        shooterSpeedLookupTable.put(2.95, 49d);
        shooterSpeedLookupTable.put(3.00, 49d);
        shooterSpeedLookupTable.put(3.05, 49.3);
        shooterSpeedLookupTable.put(3.10, 49.6);
        shooterSpeedLookupTable.put(3.15, 49.9);
        shooterSpeedLookupTable.put(3.20, 49.9);
        shooterSpeedLookupTable.put(3.25, 50.2);
        shooterSpeedLookupTable.put(3.30, 50.5);
        shooterSpeedLookupTable.put(3.35, 50.8);
        shooterSpeedLookupTable.put(3.40, 50.9);
        shooterSpeedLookupTable.put(3.45, 51.1);
        shooterSpeedLookupTable.put(3.50, 51.4);
        shooterSpeedLookupTable.put(3.55, 51.7);
        shooterSpeedLookupTable.put(3.60, 52d);
        shooterSpeedLookupTable.put(3.65, 52d);
        shooterSpeedLookupTable.put(3.70, 52.2);
        shooterSpeedLookupTable.put(3.75, 52.5);
        shooterSpeedLookupTable.put(3.80, 52.8);
        shooterSpeedLookupTable.put(3.85, 53.1);
        shooterSpeedLookupTable.put(3.90, 53.3);
        shooterSpeedLookupTable.put(3.95, 53.4);
        shooterSpeedLookupTable.put(4.00, 53.7);
        shooterSpeedLookupTable.put(4.05, 54d);
        shooterSpeedLookupTable.put(4.10, 54.3);
        shooterSpeedLookupTable.put(4.15, 54.6);
        shooterSpeedLookupTable.put(4.20, 54.9);
        shooterSpeedLookupTable.put(4.25, 54.9);
        shooterSpeedLookupTable.put(4.30, 55.2);
        shooterSpeedLookupTable.put(4.35, 55.5);
        shooterSpeedLookupTable.put(4.40, 55.8);
        shooterSpeedLookupTable.put(4.45, 56.1);
        shooterSpeedLookupTable.put(4.50, 56.2);
        shooterSpeedLookupTable.put(4.55, 56.3);
        shooterSpeedLookupTable.put(4.60, 56.6);
        shooterSpeedLookupTable.put(4.65, 56.9);
        shooterSpeedLookupTable.put(4.70, 57.2);
        shooterSpeedLookupTable.put(4.75, 57.5);
        shooterSpeedLookupTable.put(4.80, 57.7);
        shooterSpeedLookupTable.put(4.85, 57.8);
        shooterSpeedLookupTable.put(4.90, 58.1);
        shooterSpeedLookupTable.put(4.95, 58.4);
        shooterSpeedLookupTable.put(5.00, 58.7);
        shooterSpeedLookupTable.put(5.05, 58.8);
        shooterSpeedLookupTable.put(5.10, 59d);
        shooterSpeedLookupTable.put(5.15, 59.3);
        shooterSpeedLookupTable.put(5.20, 59.6);
        shooterSpeedLookupTable.put(5.25, 59.9);
        shooterSpeedLookupTable.put(5.30, 60d);
        shooterSpeedLookupTable.put(5.35, 60.2);
        shooterSpeedLookupTable.put(5.40, 60.4);
        shooterSpeedLookupTable.put(5.45, 60.7);
        shooterSpeedLookupTable.put(5.50, 61d);
        shooterSpeedLookupTable.put(5.55, 61.2);
        shooterSpeedLookupTable.put(5.60, 61.3);
        shooterSpeedLookupTable.put(5.65, 61.6);
        shooterSpeedLookupTable.put(5.70, 61.9);
        shooterSpeedLookupTable.put(5.75, 62.1);
        shooterSpeedLookupTable.put(5.80, 62.4);
        shooterSpeedLookupTable.put(5.85, 62.5);
        shooterSpeedLookupTable.put(5.90, 62.8);
        shooterSpeedLookupTable.put(5.95, 63.1);
        shooterSpeedLookupTable.put(6.00, 63.2);
        shooterSpeedLookupTable.put(6.05, 63.4);
        shooterSpeedLookupTable.put(6.10, 63.7);
        shooterSpeedLookupTable.put(6.15, 64d);
        shooterSpeedLookupTable.put(6.20, 64.1);
        shooterSpeedLookupTable.put(6.25, 64.4);
        shooterSpeedLookupTable.put(6.30, 64.6);
        shooterSpeedLookupTable.put(6.35, 64.8);
        shooterSpeedLookupTable.put(6.40, 65.1);
        shooterSpeedLookupTable.put(6.45, 65.3);
        shooterSpeedLookupTable.put(6.50, 65.4);
        shooterSpeedLookupTable.put(6.55, 65.7);
        shooterSpeedLookupTable.put(6.60, 66d);
        shooterSpeedLookupTable.put(6.65, 66.2);
        shooterSpeedLookupTable.put(6.70, 66.3);
        shooterSpeedLookupTable.put(6.75, 66.6);
        shooterSpeedLookupTable.put(6.80, 66.9);
        shooterSpeedLookupTable.put(6.85, 67d);
        shooterSpeedLookupTable.put(6.90, 67.3);
        shooterSpeedLookupTable.put(6.95, 67.5);
        shooterSpeedLookupTable.put(7.00, 67.8);
        shooterSpeedLookupTable.put(7.05, 67.9);
        shooterSpeedLookupTable.put(7.10, 68.2);
        shooterSpeedLookupTable.put(7.15, 68.4);
        shooterSpeedLookupTable.put(7.20, 68.7);
        shooterSpeedLookupTable.put(7.25, 68.8);
        shooterSpeedLookupTable.put(7.30, 69.1);
        shooterSpeedLookupTable.put(7.35, 69.2);
        shooterSpeedLookupTable.put(7.40, 69.5);
        shooterSpeedLookupTable.put(7.45, 69.7);
        shooterSpeedLookupTable.put(7.50, 70d);
        shooterSpeedLookupTable.put(7.55, 70.1);
        shooterSpeedLookupTable.put(7.60, 70.4);
        shooterSpeedLookupTable.put(7.65, 70.6);
        shooterSpeedLookupTable.put(7.70, 70.8);
        shooterSpeedLookupTable.put(7.75, 71d);
        shooterSpeedLookupTable.put(7.80, 71.3);
        shooterSpeedLookupTable.put(7.85, 71.4);
        shooterSpeedLookupTable.put(7.90, 71.7);
        shooterSpeedLookupTable.put(7.95, 71.9);
        shooterSpeedLookupTable.put(8.00, 72.2);

        timeOfFlightLookupTable.put(1.00, 0.713);
        timeOfFlightLookupTable.put(1.05, 0.729);
        timeOfFlightLookupTable.put(1.10, 0.740);
        timeOfFlightLookupTable.put(1.15, 0.750);
        timeOfFlightLookupTable.put(1.20, 0.764);
        timeOfFlightLookupTable.put(1.25, 0.772);
        timeOfFlightLookupTable.put(1.30, 0.763);
        timeOfFlightLookupTable.put(1.35, 0.775);
        timeOfFlightLookupTable.put(1.40, 0.786);
        timeOfFlightLookupTable.put(1.45, 0.796);
        timeOfFlightLookupTable.put(1.50, 0.806);
        timeOfFlightLookupTable.put(1.55, 0.815);
        timeOfFlightLookupTable.put(1.60, 0.824);
        timeOfFlightLookupTable.put(1.65, 0.814);
        timeOfFlightLookupTable.put(1.70, 0.822);
        timeOfFlightLookupTable.put(1.75, 0.833);
        timeOfFlightLookupTable.put(1.80, 0.840);
        timeOfFlightLookupTable.put(1.85, 0.851);
        timeOfFlightLookupTable.put(1.90, 0.861);
        timeOfFlightLookupTable.put(1.95, 0.867);
        timeOfFlightLookupTable.put(2.00, 0.876);
        timeOfFlightLookupTable.put(2.05, 0.886);
        timeOfFlightLookupTable.put(2.10, 0.874);
        timeOfFlightLookupTable.put(2.15, 0.886);
        timeOfFlightLookupTable.put(2.20, 0.894);
        timeOfFlightLookupTable.put(2.25, 0.883);
        timeOfFlightLookupTable.put(2.30, 0.890);
        timeOfFlightLookupTable.put(2.35, 0.898);
        timeOfFlightLookupTable.put(2.40, 0.887);
        timeOfFlightLookupTable.put(2.45, 0.894);
        timeOfFlightLookupTable.put(2.50, 0.904);
        timeOfFlightLookupTable.put(2.55, 0.913);
        timeOfFlightLookupTable.put(2.60, 0.900);
        timeOfFlightLookupTable.put(2.65, 0.905);
        timeOfFlightLookupTable.put(2.70, 0.914);
        timeOfFlightLookupTable.put(2.75, 0.923);
        timeOfFlightLookupTable.put(2.80, 0.907);
        timeOfFlightLookupTable.put(2.85, 0.915);
        timeOfFlightLookupTable.put(2.90, 0.926);
        timeOfFlightLookupTable.put(2.95, 0.934);
        timeOfFlightLookupTable.put(3.00, 0.915);
        timeOfFlightLookupTable.put(3.05, 0.926);
        timeOfFlightLookupTable.put(3.10, 0.936);
        timeOfFlightLookupTable.put(3.15, 0.946);
        timeOfFlightLookupTable.put(3.20, 0.919);
        timeOfFlightLookupTable.put(3.25, 0.929);
        timeOfFlightLookupTable.put(3.30, 0.939);
        timeOfFlightLookupTable.put(3.35, 0.948);
        timeOfFlightLookupTable.put(3.40, 0.925);
        timeOfFlightLookupTable.put(3.45, 0.934);
        timeOfFlightLookupTable.put(3.50, 0.943);
        timeOfFlightLookupTable.put(3.55, 0.952);
        timeOfFlightLookupTable.put(3.60, 0.961);
        timeOfFlightLookupTable.put(3.65, 0.926);
        timeOfFlightLookupTable.put(3.70, 0.934);
        timeOfFlightLookupTable.put(3.75, 0.942);
        timeOfFlightLookupTable.put(3.80, 0.950);
        timeOfFlightLookupTable.put(3.85, 0.958);
        timeOfFlightLookupTable.put(3.90, 0.933);
        timeOfFlightLookupTable.put(3.95, 0.943);
        timeOfFlightLookupTable.put(4.00, 0.950);
        timeOfFlightLookupTable.put(4.05, 0.958);
        timeOfFlightLookupTable.put(4.10, 0.965);
        timeOfFlightLookupTable.put(4.15, 0.972);
        timeOfFlightLookupTable.put(4.20, 0.980);
        timeOfFlightLookupTable.put(4.25, 0.971);
        timeOfFlightLookupTable.put(4.30, 0.978);
        timeOfFlightLookupTable.put(4.35, 0.984);
        timeOfFlightLookupTable.put(4.40, 0.991);
        timeOfFlightLookupTable.put(4.45, 0.998);
        timeOfFlightLookupTable.put(4.50, 1.003);
        timeOfFlightLookupTable.put(4.55, 1.007);
        timeOfFlightLookupTable.put(4.60, 1.014);
        timeOfFlightLookupTable.put(4.65, 1.020);
        timeOfFlightLookupTable.put(4.70, 1.027);
        timeOfFlightLookupTable.put(4.75, 1.033);
        timeOfFlightLookupTable.put(4.80, 1.042);
        timeOfFlightLookupTable.put(4.85, 1.051);
        timeOfFlightLookupTable.put(4.90, 1.057);
        timeOfFlightLookupTable.put(4.95, 1.064);
        timeOfFlightLookupTable.put(5.00, 1.070);
        timeOfFlightLookupTable.put(5.05, 1.071);
        timeOfFlightLookupTable.put(5.10, 1.070);
        timeOfFlightLookupTable.put(5.15, 1.076);
        timeOfFlightLookupTable.put(5.20, 1.082);
        timeOfFlightLookupTable.put(5.25, 1.088);
        timeOfFlightLookupTable.put(5.30, 1.096);
        timeOfFlightLookupTable.put(5.35, 1.097);
        timeOfFlightLookupTable.put(5.40, 1.103);
        timeOfFlightLookupTable.put(5.45, 1.108);
        timeOfFlightLookupTable.put(5.50, 1.114);
        timeOfFlightLookupTable.put(5.55, 1.122);
        timeOfFlightLookupTable.put(5.60, 1.125);
        timeOfFlightLookupTable.put(5.65, 1.131);
        timeOfFlightLookupTable.put(5.70, 1.136);
        timeOfFlightLookupTable.put(5.75, 1.129);
        timeOfFlightLookupTable.put(5.80, 1.134);
        timeOfFlightLookupTable.put(5.85, 1.142);
        timeOfFlightLookupTable.put(5.90, 1.148);
        timeOfFlightLookupTable.put(5.95, 1.153);
        timeOfFlightLookupTable.put(6.00, 1.161);
        timeOfFlightLookupTable.put(6.05, 1.166);
        timeOfFlightLookupTable.put(6.10, 1.171);
        timeOfFlightLookupTable.put(6.15, 1.176);
        timeOfFlightLookupTable.put(6.20, 1.181);
        timeOfFlightLookupTable.put(6.25, 1.186);
        timeOfFlightLookupTable.put(6.30, 1.194);
        timeOfFlightLookupTable.put(6.35, 1.199);
        timeOfFlightLookupTable.put(6.40, 1.204);
        timeOfFlightLookupTable.put(6.45, 1.211);
        timeOfFlightLookupTable.put(6.50, 1.219);
        timeOfFlightLookupTable.put(6.55, 1.224);
        timeOfFlightLookupTable.put(6.60, 1.228);
        timeOfFlightLookupTable.put(6.65, 1.236);
        timeOfFlightLookupTable.put(6.70, 1.235);
        timeOfFlightLookupTable.put(6.75, 1.240);
        timeOfFlightLookupTable.put(6.80, 1.245);
        timeOfFlightLookupTable.put(6.85, 1.252);
        timeOfFlightLookupTable.put(6.90, 1.256);
        timeOfFlightLookupTable.put(6.95, 1.264);
        timeOfFlightLookupTable.put(7.00, 1.268);
        timeOfFlightLookupTable.put(7.05, 1.276);
        timeOfFlightLookupTable.put(7.10, 1.280);
        timeOfFlightLookupTable.put(7.15, 1.287);
        timeOfFlightLookupTable.put(7.20, 1.292);
        timeOfFlightLookupTable.put(7.25, 1.299);
        timeOfFlightLookupTable.put(7.30, 1.303);
        timeOfFlightLookupTable.put(7.35, 1.310);
        timeOfFlightLookupTable.put(7.40, 1.315);
        timeOfFlightLookupTable.put(7.45, 1.322);
        timeOfFlightLookupTable.put(7.50, 1.326);
        timeOfFlightLookupTable.put(7.55, 1.333);
        timeOfFlightLookupTable.put(7.60, 1.337);
        timeOfFlightLookupTable.put(7.65, 1.344);
        timeOfFlightLookupTable.put(7.70, 1.349);
        timeOfFlightLookupTable.put(7.75, 1.356);
        timeOfFlightLookupTable.put(7.80, 1.360);
        timeOfFlightLookupTable.put(7.85, 1.367);
        timeOfFlightLookupTable.put(7.90, 1.371);
        timeOfFlightLookupTable.put(7.95, 1.378);
        timeOfFlightLookupTable.put(8.00, 1.382);


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
        shuttleShooterSpeedLookupTable.put(7.00, 77.0);
        shuttleShooterSpeedLookupTable.put(7.50, 80.2);
        shuttleShooterSpeedLookupTable.put(8.00, 83.3);
        shuttleShooterSpeedLookupTable.put(8.50, 86.3);
        shuttleShooterSpeedLookupTable.put(9.00, 89.2);
        shuttleShooterSpeedLookupTable.put(9.50, 92.2);
        shuttleShooterSpeedLookupTable.put(10.00, 95.0);
        shuttleShooterSpeedLookupTable.put(10.50, 97.8);
        shuttleShooterSpeedLookupTable.put(11.00, 97.8);

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
        shuttleTimeOfFlightLookupTable.put(8.00, 1.518);
        shuttleTimeOfFlightLookupTable.put(8.50, 1.569);
        shuttleTimeOfFlightLookupTable.put(9.00, 1.619);
        shuttleTimeOfFlightLookupTable.put(9.50, 1.667);
        shuttleTimeOfFlightLookupTable.put(10.00, 1.716);
        shuttleTimeOfFlightLookupTable.put(10.50, 1.764);
        shuttleTimeOfFlightLookupTable.put(11.00, 1.764);


//        lastTurretAngle = turretSubsystem.getDegrees();
//        lastTurretAnglePhaseDelayed = turretSubsystem.getDegrees();
    }


    public static ShotCalculator getInstance() {
        if (shotCalculator == null) shotCalculator = new ShotCalculator();
        return shotCalculator;
    }

    public void periodic() {
        if (RobotContainer.getPose() != null) {
            Pose2d estimatedPose = RobotContainer.getPose();
            Pose2d estimatedPosePhaseDelayed =
                    estimatedPose.plus(
                            new Twist2d(
                                    RobotContainer.getVelocity().vx * Constants.PHASE_DELAY
                                            + RobotContainer.getAccelerationX() * Constants.ACCELERATION_PHASE_DELAY,
                                    RobotContainer.getVelocity().vy * Constants.PHASE_DELAY
                                            + RobotContainer.getAccelerationY() * Constants.ACCELERATION_PHASE_DELAY,
                                    RobotContainer.getVelocity().omega * Constants.ROTATIONAL_PHASE_DELAY
                                            + RobotContainer.getAccelerationOmega() * Constants.ACCELERATION_PHASE_DELAY).exp());

            // System.out.println(estimatedPosePhaseDelayed);
            Translation2d target = getCurrentTarget();
            ChassisVelocities fieldRelativeVelocities = RobotContainer.getVelocity()
                    .toFieldRelative(RobotContainer.getPose().getRotation());


            Pair<Pose2d, Double> turretPhaseDelayed = createFutureTurretPose(estimatedPosePhaseDelayed, fieldRelativeVelocities, target);
            Pair<Pose2d, Double> turret = createFutureTurretPose(estimatedPose, fieldRelativeVelocities, target);

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

            turretRotationPhaseDelayed = target.minus(futureTurretPositionPhaseDelayed.getTranslation()).getAngle().orElse(lastTurretRotationPhaseDelayed);

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

            turretVelocityPhaseDelayed -= (Math.toDegrees(RobotContainer.getVelocity().omega));

            //Turret Angle Calculations
            turretRotation = target.minus(futureTurretPosition.getTranslation()).getAngle().orElse(lastTurretRotation);

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


        //Offset for left corner
//        if (Robot.getAlliance() != null) {
//            if (futureTurretToTargetDistance >= 4) {
//                if ((Robot.getAlliance().equals(Alliance.RED)
//                        && futureTurretPosition.getY() < Constants.RED_HUB_CENTER.getY())
//                        || (Robot.getAlliance().equals(Alliance.BLUE)
//                        && futureTurretPosition.getY() > Constants.BLUE_HUB_CENTER.getY())) {
//                    turretAngle += .25;
//                    turretAnglePhaseDelayed += .25;
//                    shooterSpeed += .25;
//                    shooterSpeedPhaseDelayed += .25;
//                }
//            }
        if (futureTurretToTargetDistance >= 4.75) {
            if ((Robot.getAlliance().equals(Alliance.RED)
                    && futureTurretPosition.getY() < Constants.RED_HUB_CENTER.getY())
                    || (Robot.getAlliance().equals(Alliance.BLUE)
                    && futureTurretPosition.getY() > Constants.BLUE_HUB_CENTER.getY())) {
                turretAngle += 1d;
                turretAnglePhaseDelayed += 1d;
            }
        }
//            if (futureTurretToTargetDistance >= 4.65) {
//                if ((Robot.getAlliance().equals(Alliance.RED)
//                        && futureTurretPosition.getY() > Constants.RED_HUB_CENTER.getY())
//                        || (Robot.getAlliance().equals(Alliance.BLUE)
//                        && futureTurretPosition.getY() < Constants.BLUE_HUB_CENTER.getY())) {
//                    shooterSpeed += .75;
//                    shooterSpeedPhaseDelayed += .75;
//                }
//            }
//        }

        turretSubsystem.updateGoalPosition(getTurretAnglePhaseDelayed(), getTurretVelocityPhaseDelayed());
        RobotContainer.getShooterControlAuto().setGoal(shotCalculator.getShooterSpeedPhaseDelayed());
        RobotContainer.getShooterControlAutoRB().setGoal(shotCalculator.getShooterSpeedPhaseDelayed());
        hoodSubsystem.updateGoalPosition(getHoodAnglePhaseDelayed(), getHoodVelocityPhaseDelayed());

    }


    public Pair<Pose2d, Double> createFutureTurretPose(Pose2d pose2d, ChassisVelocities fieldRelativeVelocity, Translation2d target) {
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
                fieldRelativeVelocity.vx
                        + fieldRelativeVelocity.omega
                        * (-Constants.ROBOT_TO_TURRET.getY() * Math.cos(robotAngleRadians)
                        - Constants.ROBOT_TO_TURRET.getX() * Math.sin(robotAngleRadians));
        double turretVelocityY =
                fieldRelativeVelocity.vy
                        + fieldRelativeVelocity.omega
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
        return futureTurretToTargetDistance >= 1d && futureTurretToTargetDistance <= 8d;
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