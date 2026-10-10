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

        hoodLookupTable.put(1.00, 0.2d);
        hoodLookupTable.put(1.05, 1.3d);
        hoodLookupTable.put(1.10, 1.7d);
        hoodLookupTable.put(1.15, 2.0d);
        hoodLookupTable.put(1.20, 2.4d);
        hoodLookupTable.put(1.25, 2.7d);
        hoodLookupTable.put(1.30, 3.7d);
        hoodLookupTable.put(1.35, 4.1d);
        hoodLookupTable.put(1.40, 4.3d);
        hoodLookupTable.put(1.45, 4.6d);
        hoodLookupTable.put(1.50, 4.9d);
        hoodLookupTable.put(1.55, 6.0d);
        hoodLookupTable.put(1.60, 6.2d);
        hoodLookupTable.put(1.65, 6.5d);
        hoodLookupTable.put(1.70, 6.7d);
        hoodLookupTable.put(1.75, 6.9d);
        hoodLookupTable.put(1.80, 7.2d);
        hoodLookupTable.put(1.85, 8.2d);
        hoodLookupTable.put(1.90, 8.4d);
        hoodLookupTable.put(1.95, 8.6d);
        hoodLookupTable.put(2.00, 8.8d);
        hoodLookupTable.put(2.05, 9.0d);
        hoodLookupTable.put(2.10, 9.2d);
        hoodLookupTable.put(2.15, 10.3d);
        hoodLookupTable.put(2.20, 10.4d);
        hoodLookupTable.put(2.25, 10.6d);
        hoodLookupTable.put(2.30, 10.7d);
        hoodLookupTable.put(2.35, 11.9d);
        hoodLookupTable.put(2.40, 12.0d);
        hoodLookupTable.put(2.45, 12.1d);
        hoodLookupTable.put(2.50, 12.2d);
        hoodLookupTable.put(2.55, 12.3d);
        hoodLookupTable.put(2.60, 13.6d);
        hoodLookupTable.put(2.65, 13.7d);
        hoodLookupTable.put(2.70, 13.7d);
        hoodLookupTable.put(2.75, 13.8d);
        hoodLookupTable.put(2.80, 13.8d);
        hoodLookupTable.put(2.85, 15.3d);
        hoodLookupTable.put(2.90, 15.3d);
        hoodLookupTable.put(2.95, 15.3d);
        hoodLookupTable.put(3.00, 15.3d);
        hoodLookupTable.put(3.05, 15.3d);
        hoodLookupTable.put(3.10, 16.9d);
        hoodLookupTable.put(3.15, 16.9d);
        hoodLookupTable.put(3.20, 16.9d);
        hoodLookupTable.put(3.25, 16.9d);
        hoodLookupTable.put(3.30, 16.9d);
        hoodLookupTable.put(3.35, 18.6d);
        hoodLookupTable.put(3.40, 18.6d);
        hoodLookupTable.put(3.45, 18.6d);
        hoodLookupTable.put(3.50, 18.6d);
        hoodLookupTable.put(3.55, 18.6d);
        hoodLookupTable.put(3.60, 20.0d);
        hoodLookupTable.put(3.65, 20.0d);
        hoodLookupTable.put(3.70, 20.0d);
        hoodLookupTable.put(3.75, 20.0d);
        hoodLookupTable.put(3.80, 20.0d);
        hoodLookupTable.put(3.85, 20.0d);
        hoodLookupTable.put(3.90, 21.0d);
        hoodLookupTable.put(3.95, 21.0d);
        hoodLookupTable.put(4.00, 21.0d);
        hoodLookupTable.put(4.05, 21.0d);
        hoodLookupTable.put(4.10, 21.0d);
        hoodLookupTable.put(4.15, 21.0d);
        hoodLookupTable.put(4.20, 21.5d);
        hoodLookupTable.put(4.25, 21.6d);
        hoodLookupTable.put(4.30, 21.6d);
        hoodLookupTable.put(4.35, 21.6d);
        hoodLookupTable.put(4.40, 21.6d);
        hoodLookupTable.put(4.45, 21.6d);
        hoodLookupTable.put(4.50, 21.6d);
        hoodLookupTable.put(4.55, 21.8d);
        hoodLookupTable.put(4.60, 21.8d);
        hoodLookupTable.put(4.65, 21.8d);
        hoodLookupTable.put(4.70, 21.8d);
        hoodLookupTable.put(4.75, 22.2d);
        hoodLookupTable.put(4.80, 22.5d);
        hoodLookupTable.put(4.85, 22.5d);
        hoodLookupTable.put(4.90, 22.5d);
        hoodLookupTable.put(4.95, 22.5d);
        hoodLookupTable.put(5.00, 22.6d);
        hoodLookupTable.put(5.05, 23.0d);
        hoodLookupTable.put(5.10, 23.0d);
        hoodLookupTable.put(5.15, 23.0d);
        hoodLookupTable.put(5.20, 23.0d);
        hoodLookupTable.put(5.25, 23.0d);
        hoodLookupTable.put(5.30, 23.2d);
        hoodLookupTable.put(5.35, 23.2d);
        hoodLookupTable.put(5.40, 23.2d);
        hoodLookupTable.put(5.45, 23.9d);
        hoodLookupTable.put(5.50, 24.5d);
        hoodLookupTable.put(5.55, 24.5d);
        hoodLookupTable.put(5.60, 24.5d);
        hoodLookupTable.put(5.65, 24.5d);
        hoodLookupTable.put(5.70, 24.5d);
        hoodLookupTable.put(5.75, 24.5d);
        hoodLookupTable.put(5.80, 24.5d);
        hoodLookupTable.put(5.85, 24.5d);
        hoodLookupTable.put(5.90, 24.5d);
        hoodLookupTable.put(5.95, 24.8d);
        hoodLookupTable.put(6.00, 24.8d);
        hoodLookupTable.put(6.05, 24.8d);
        hoodLookupTable.put(6.10, 24.8d);
        hoodLookupTable.put(6.15, 24.8d);
        hoodLookupTable.put(6.20, 24.8d);
        hoodLookupTable.put(6.25, 24.8d);
        hoodLookupTable.put(6.30, 24.8d);
        hoodLookupTable.put(6.35, 24.8d);
        hoodLookupTable.put(6.40, 24.8d);
        hoodLookupTable.put(6.45, 24.8d);
        hoodLookupTable.put(6.50, 24.8d);
        hoodLookupTable.put(6.55, 24.8d);
        hoodLookupTable.put(6.60, 24.8d);
        hoodLookupTable.put(6.65, 24.8d);
        hoodLookupTable.put(6.70, 24.8d);
        hoodLookupTable.put(6.75, 24.8d);
        hoodLookupTable.put(6.80, 24.9d);
        hoodLookupTable.put(6.85, 24.9d);
        hoodLookupTable.put(6.90, 24.9d);
        hoodLookupTable.put(6.95, 24.9d);
        hoodLookupTable.put(7.00, 25.0d);
        hoodLookupTable.put(7.05, 25.0d);
        hoodLookupTable.put(7.10, 25.0d);
        hoodLookupTable.put(7.15, 25.0d);
        hoodLookupTable.put(7.20, 25.0d);
        hoodLookupTable.put(7.25, 25.0d);
        hoodLookupTable.put(7.30, 25.0d);
        hoodLookupTable.put(7.35, 25.0d);
        hoodLookupTable.put(7.40, 25.0d);
        hoodLookupTable.put(7.45, 25.0d);
        hoodLookupTable.put(7.50, 25.0d);
        hoodLookupTable.put(7.55, 25.0d);
        hoodLookupTable.put(7.60, 25.0d);
        hoodLookupTable.put(7.65, 25.0d);
        hoodLookupTable.put(7.70, 25.0d);
        hoodLookupTable.put(7.75, 25.0d);
        hoodLookupTable.put(7.80, 25.0d);
        hoodLookupTable.put(7.85, 25.0d);
        hoodLookupTable.put(7.90, 25.0d);
        hoodLookupTable.put(7.95, 25.0d);
        hoodLookupTable.put(8.00, 25.0d);

        shooterSpeedLookupTable.put(1.00, 39.1);
        shooterSpeedLookupTable.put(1.05, 39.1);
        shooterSpeedLookupTable.put(1.10, 39.4);
        shooterSpeedLookupTable.put(1.15, 39.6);
        shooterSpeedLookupTable.put(1.20, 39.9);
        shooterSpeedLookupTable.put(1.25, 40.2);
        shooterSpeedLookupTable.put(1.30, 40.2);
        shooterSpeedLookupTable.put(1.35, 40.5);
        shooterSpeedLookupTable.put(1.40, 40.8);
        shooterSpeedLookupTable.put(1.45, 41.1);
        shooterSpeedLookupTable.put(1.50, 41.4);
        shooterSpeedLookupTable.put(1.55, 41.4);
        shooterSpeedLookupTable.put(1.60, 41.7);
        shooterSpeedLookupTable.put(1.65, 42d);
        shooterSpeedLookupTable.put(1.70, 42.3);
        shooterSpeedLookupTable.put(1.75, 42.6);
        shooterSpeedLookupTable.put(1.80, 42.9);
        shooterSpeedLookupTable.put(1.85, 42.9);
        shooterSpeedLookupTable.put(1.90, 43.2);
        shooterSpeedLookupTable.put(1.95, 43.5);
        shooterSpeedLookupTable.put(2.00, 43.8);
        shooterSpeedLookupTable.put(2.05, 44d);
        shooterSpeedLookupTable.put(2.10, 44.3);
        shooterSpeedLookupTable.put(2.15, 44.3);
        shooterSpeedLookupTable.put(2.20, 44.6);
        shooterSpeedLookupTable.put(2.25, 44.9);
        shooterSpeedLookupTable.put(2.30, 45.2);
        shooterSpeedLookupTable.put(2.35, 45.2);
        shooterSpeedLookupTable.put(2.40, 45.5);
        shooterSpeedLookupTable.put(2.45, 45.8);
        shooterSpeedLookupTable.put(2.50, 46.1);
        shooterSpeedLookupTable.put(2.55, 46.4);
        shooterSpeedLookupTable.put(2.60, 46.4);
        shooterSpeedLookupTable.put(2.65, 46.7);
        shooterSpeedLookupTable.put(2.70, 47d);
        shooterSpeedLookupTable.put(2.75, 47.3);
        shooterSpeedLookupTable.put(2.80, 47.6);
        shooterSpeedLookupTable.put(2.85, 47.6);
        shooterSpeedLookupTable.put(2.90, 47.9);
        shooterSpeedLookupTable.put(2.95, 48.1);
        shooterSpeedLookupTable.put(3.00, 48.4);
        shooterSpeedLookupTable.put(3.05, 48.7);
        shooterSpeedLookupTable.put(3.10, 48.7);
        shooterSpeedLookupTable.put(3.15, 49d);
        shooterSpeedLookupTable.put(3.20, 49.3);
        shooterSpeedLookupTable.put(3.25, 49.6);
        shooterSpeedLookupTable.put(3.30, 49.9);
        shooterSpeedLookupTable.put(3.35, 49.9);
        shooterSpeedLookupTable.put(3.40, 50.2);
        shooterSpeedLookupTable.put(3.45, 50.5);
        shooterSpeedLookupTable.put(3.50, 50.8);
        shooterSpeedLookupTable.put(3.55, 51.1);
        shooterSpeedLookupTable.put(3.60, 51.2);
        shooterSpeedLookupTable.put(3.65, 51.4);
        shooterSpeedLookupTable.put(3.70, 51.7);
        shooterSpeedLookupTable.put(3.75, 52d);
        shooterSpeedLookupTable.put(3.80, 52.2);
        shooterSpeedLookupTable.put(3.85, 52.5);
        shooterSpeedLookupTable.put(3.90, 52.7);
        shooterSpeedLookupTable.put(3.95, 52.8);
        shooterSpeedLookupTable.put(4.00, 53.1);
        shooterSpeedLookupTable.put(4.05, 53.4);
        shooterSpeedLookupTable.put(4.10, 53.7);
        shooterSpeedLookupTable.put(4.15, 54d);
        shooterSpeedLookupTable.put(4.20, 54.2);
        shooterSpeedLookupTable.put(4.25, 54.3);
        shooterSpeedLookupTable.put(4.30, 54.6);
        shooterSpeedLookupTable.put(4.35, 54.9);
        shooterSpeedLookupTable.put(4.40, 55.2);
        shooterSpeedLookupTable.put(4.45, 55.5);
        shooterSpeedLookupTable.put(4.50, 55.6);
        shooterSpeedLookupTable.put(4.55, 55.8);
        shooterSpeedLookupTable.put(4.60, 56.1);
        shooterSpeedLookupTable.put(4.65, 56.3);
        shooterSpeedLookupTable.put(4.70, 56.6);
        shooterSpeedLookupTable.put(4.75, 56.8);
        shooterSpeedLookupTable.put(4.80, 56.9);
        shooterSpeedLookupTable.put(4.85, 57.2);
        shooterSpeedLookupTable.put(4.90, 57.5);
        shooterSpeedLookupTable.put(4.95, 57.8);
        shooterSpeedLookupTable.put(5.00, 58d);
        shooterSpeedLookupTable.put(5.05, 58.1);
        shooterSpeedLookupTable.put(5.10, 58.4);
        shooterSpeedLookupTable.put(5.15, 58.7);
        shooterSpeedLookupTable.put(5.20, 59d);
        shooterSpeedLookupTable.put(5.25, 59.1);
        shooterSpeedLookupTable.put(5.30, 59.3);
        shooterSpeedLookupTable.put(5.35, 59.6);
        shooterSpeedLookupTable.put(5.40, 59.9);
        shooterSpeedLookupTable.put(5.45, 60d);
        shooterSpeedLookupTable.put(5.50, 60.2);
        shooterSpeedLookupTable.put(5.55, 60.4);
        shooterSpeedLookupTable.put(5.60, 60.7);
        shooterSpeedLookupTable.put(5.65, 61d);
        shooterSpeedLookupTable.put(5.70, 61.2);
        shooterSpeedLookupTable.put(5.75, 61.3);
        shooterSpeedLookupTable.put(5.80, 61.6);
        shooterSpeedLookupTable.put(5.85, 61.9);
        shooterSpeedLookupTable.put(5.90, 62.1);
        shooterSpeedLookupTable.put(5.95, 62.2);
        shooterSpeedLookupTable.put(6.00, 62.5);
        shooterSpeedLookupTable.put(6.05, 62.8);
        shooterSpeedLookupTable.put(6.10, 62.9);
        shooterSpeedLookupTable.put(6.15, 63.2);
        shooterSpeedLookupTable.put(6.20, 63.4);
        shooterSpeedLookupTable.put(6.25, 63.7);
        shooterSpeedLookupTable.put(6.30, 64d);
        shooterSpeedLookupTable.put(6.35, 64.1);
        shooterSpeedLookupTable.put(6.40, 64.3);
        shooterSpeedLookupTable.put(6.45, 64.6);
        shooterSpeedLookupTable.put(6.50, 64.8);
        shooterSpeedLookupTable.put(6.55, 65d);
        shooterSpeedLookupTable.put(6.60, 65.1);
        shooterSpeedLookupTable.put(6.65, 65.4);
        shooterSpeedLookupTable.put(6.70, 65.7);
        shooterSpeedLookupTable.put(6.75, 65.9);
        shooterSpeedLookupTable.put(6.80, 66d);
        shooterSpeedLookupTable.put(6.85, 66.3);
        shooterSpeedLookupTable.put(6.90, 66.6);
        shooterSpeedLookupTable.put(6.95, 66.7);
        shooterSpeedLookupTable.put(7.00, 66.9);
        shooterSpeedLookupTable.put(7.05, 67.2);
        shooterSpeedLookupTable.put(7.10, 67.5);
        shooterSpeedLookupTable.put(7.15, 67.6);
        shooterSpeedLookupTable.put(7.20, 67.8);
        shooterSpeedLookupTable.put(7.25, 68.1);
        shooterSpeedLookupTable.put(7.30, 68.4);
        shooterSpeedLookupTable.put(7.35, 68.5);
        shooterSpeedLookupTable.put(7.40, 68.7);
        shooterSpeedLookupTable.put(7.45, 68.9);
        shooterSpeedLookupTable.put(7.50, 69.2);
        shooterSpeedLookupTable.put(7.55, 69.4);
        shooterSpeedLookupTable.put(7.60, 69.5);
        shooterSpeedLookupTable.put(7.65, 69.8);
        shooterSpeedLookupTable.put(7.70, 70d);
        shooterSpeedLookupTable.put(7.75, 70.3);
        shooterSpeedLookupTable.put(7.80, 70.4);
        shooterSpeedLookupTable.put(7.85, 70.7);
        shooterSpeedLookupTable.put(7.90, 70.8);
        shooterSpeedLookupTable.put(7.95, 71.1);
        shooterSpeedLookupTable.put(8.00, 71.3);

        timeOfFlightLookupTable.put(1.00, 0.706);
        timeOfFlightLookupTable.put(1.05, 0.692);
        timeOfFlightLookupTable.put(1.10, 0.704);
        timeOfFlightLookupTable.put(1.15, 0.718);
        timeOfFlightLookupTable.put(1.20, 0.728);
        timeOfFlightLookupTable.put(1.25, 0.741);
        timeOfFlightLookupTable.put(1.30, 0.731);
        timeOfFlightLookupTable.put(1.35, 0.738);
        timeOfFlightLookupTable.put(1.40, 0.753);
        timeOfFlightLookupTable.put(1.45, 0.764);
        timeOfFlightLookupTable.put(1.50, 0.774);
        timeOfFlightLookupTable.put(1.55, 0.759);
        timeOfFlightLookupTable.put(1.60, 0.772);
        timeOfFlightLookupTable.put(1.65, 0.780);
        timeOfFlightLookupTable.put(1.70, 0.792);
        timeOfFlightLookupTable.put(1.75, 0.803);
        timeOfFlightLookupTable.put(1.80, 0.811);
        timeOfFlightLookupTable.put(1.85, 0.799);
        timeOfFlightLookupTable.put(1.90, 0.809);
        timeOfFlightLookupTable.put(1.95, 0.819);
        timeOfFlightLookupTable.put(2.00, 0.828);
        timeOfFlightLookupTable.put(2.05, 0.837);
        timeOfFlightLookupTable.put(2.10, 0.846);
        timeOfFlightLookupTable.put(2.15, 0.831);
        timeOfFlightLookupTable.put(2.20, 0.842);
        timeOfFlightLookupTable.put(2.25, 0.850);
        timeOfFlightLookupTable.put(2.30, 0.861);
        timeOfFlightLookupTable.put(2.35, 0.843);
        timeOfFlightLookupTable.put(2.40, 0.853);
        timeOfFlightLookupTable.put(2.45, 0.863);
        timeOfFlightLookupTable.put(2.50, 0.872);
        timeOfFlightLookupTable.put(2.55, 0.882);
        timeOfFlightLookupTable.put(2.60, 0.861);
        timeOfFlightLookupTable.put(2.65, 0.870);
        timeOfFlightLookupTable.put(2.70, 0.882);
        timeOfFlightLookupTable.put(2.75, 0.890);
        timeOfFlightLookupTable.put(2.80, 0.902);
        timeOfFlightLookupTable.put(2.85, 0.876);
        timeOfFlightLookupTable.put(2.90, 0.886);
        timeOfFlightLookupTable.put(2.95, 0.897);
        timeOfFlightLookupTable.put(3.00, 0.907);
        timeOfFlightLookupTable.put(3.05, 0.918);
        timeOfFlightLookupTable.put(3.10, 0.890);
        timeOfFlightLookupTable.put(3.15, 0.900);
        timeOfFlightLookupTable.put(3.20, 0.909);
        timeOfFlightLookupTable.put(3.25, 0.919);
        timeOfFlightLookupTable.put(3.30, 0.928);
        timeOfFlightLookupTable.put(3.35, 0.899);
        timeOfFlightLookupTable.put(3.40, 0.908);
        timeOfFlightLookupTable.put(3.45, 0.917);
        timeOfFlightLookupTable.put(3.50, 0.925);
        timeOfFlightLookupTable.put(3.55, 0.934);
        timeOfFlightLookupTable.put(3.60, 0.911);
        timeOfFlightLookupTable.put(3.65, 0.922);
        timeOfFlightLookupTable.put(3.70, 0.930);
        timeOfFlightLookupTable.put(3.75, 0.938);
        timeOfFlightLookupTable.put(3.80, 0.946);
        timeOfFlightLookupTable.put(3.85, 0.953);
        timeOfFlightLookupTable.put(3.90, 0.940);
        timeOfFlightLookupTable.put(3.95, 0.950);
        timeOfFlightLookupTable.put(4.00, 0.957);
        timeOfFlightLookupTable.put(4.05, 0.965);
        timeOfFlightLookupTable.put(4.10, 0.972);
        timeOfFlightLookupTable.put(4.15, 0.979);
        timeOfFlightLookupTable.put(4.20, 0.977);
        timeOfFlightLookupTable.put(4.25, 0.984);
        timeOfFlightLookupTable.put(4.30, 0.991);
        timeOfFlightLookupTable.put(4.35, 0.998);
        timeOfFlightLookupTable.put(4.40, 1.005);
        timeOfFlightLookupTable.put(4.45, 1.012);
        timeOfFlightLookupTable.put(4.50, 1.021);
        timeOfFlightLookupTable.put(4.55, 1.026);
        timeOfFlightLookupTable.put(4.60, 1.032);
        timeOfFlightLookupTable.put(4.65, 1.039);
        timeOfFlightLookupTable.put(4.70, 1.046);
        timeOfFlightLookupTable.put(4.75, 1.045);
        timeOfFlightLookupTable.put(4.80, 1.046);
        timeOfFlightLookupTable.put(4.85, 1.052);
        timeOfFlightLookupTable.put(4.90, 1.059);
        timeOfFlightLookupTable.put(4.95, 1.065);
        timeOfFlightLookupTable.put(5.00, 1.071);
        timeOfFlightLookupTable.put(5.05, 1.070);
        timeOfFlightLookupTable.put(5.10, 1.076);
        timeOfFlightLookupTable.put(5.15, 1.082);
        timeOfFlightLookupTable.put(5.20, 1.088);
        timeOfFlightLookupTable.put(5.25, 1.096);
        timeOfFlightLookupTable.put(5.30, 1.100);
        timeOfFlightLookupTable.put(5.35, 1.105);
        timeOfFlightLookupTable.put(5.40, 1.111);
        timeOfFlightLookupTable.put(5.45, 1.102);
        timeOfFlightLookupTable.put(5.50, 1.095);
        timeOfFlightLookupTable.put(5.55, 1.101);
        timeOfFlightLookupTable.put(5.60, 1.106);
        timeOfFlightLookupTable.put(5.65, 1.111);
        timeOfFlightLookupTable.put(5.70, 1.119);
        timeOfFlightLookupTable.put(5.75, 1.127);
        timeOfFlightLookupTable.put(5.80, 1.133);
        timeOfFlightLookupTable.put(5.85, 1.138);
        timeOfFlightLookupTable.put(5.90, 1.146);
        timeOfFlightLookupTable.put(5.95, 1.146);
        timeOfFlightLookupTable.put(6.00, 1.151);
        timeOfFlightLookupTable.put(6.05, 1.156);
        timeOfFlightLookupTable.put(6.10, 1.164);
        timeOfFlightLookupTable.put(6.15, 1.169);
        timeOfFlightLookupTable.put(6.20, 1.176);
        timeOfFlightLookupTable.put(6.25, 1.181);
        timeOfFlightLookupTable.put(6.30, 1.186);
        timeOfFlightLookupTable.put(6.35, 1.194);
        timeOfFlightLookupTable.put(6.40, 1.201);
        timeOfFlightLookupTable.put(6.45, 1.206);
        timeOfFlightLookupTable.put(6.50, 1.211);
        timeOfFlightLookupTable.put(6.55, 1.218);
        timeOfFlightLookupTable.put(6.60, 1.226);
        timeOfFlightLookupTable.put(6.65, 1.230);
        timeOfFlightLookupTable.put(6.70, 1.235);
        timeOfFlightLookupTable.put(6.75, 1.242);
        timeOfFlightLookupTable.put(6.80, 1.247);
        timeOfFlightLookupTable.put(6.85, 1.252);
        timeOfFlightLookupTable.put(6.90, 1.256);
        timeOfFlightLookupTable.put(6.95, 1.263);
        timeOfFlightLookupTable.put(7.00, 1.268);
        timeOfFlightLookupTable.put(7.05, 1.272);
        timeOfFlightLookupTable.put(7.10, 1.277);
        timeOfFlightLookupTable.put(7.15, 1.284);
        timeOfFlightLookupTable.put(7.20, 1.291);
        timeOfFlightLookupTable.put(7.25, 1.295);
        timeOfFlightLookupTable.put(7.30, 1.300);
        timeOfFlightLookupTable.put(7.35, 1.307);
        timeOfFlightLookupTable.put(7.40, 1.314);
        timeOfFlightLookupTable.put(7.45, 1.318);
        timeOfFlightLookupTable.put(7.50, 1.322);
        timeOfFlightLookupTable.put(7.55, 1.329);
        timeOfFlightLookupTable.put(7.60, 1.336);
        timeOfFlightLookupTable.put(7.65, 1.341);
        timeOfFlightLookupTable.put(7.70, 1.347);
        timeOfFlightLookupTable.put(7.75, 1.352);
        timeOfFlightLookupTable.put(7.80, 1.359);
        timeOfFlightLookupTable.put(7.85, 1.363);
        timeOfFlightLookupTable.put(7.90, 1.369);
        timeOfFlightLookupTable.put(7.95, 1.374);
        timeOfFlightLookupTable.put(8.00, 1.380);


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