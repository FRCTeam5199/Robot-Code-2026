package frc.robot.utility;

import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
import frc.robot.constants.TurretConstants;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.TurretSubsystem;
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

        hoodLookupTable.put(1.10, 0.0d);
        hoodLookupTable.put(1.15, 0.4d);
        hoodLookupTable.put(1.20, 0.7d);
        hoodLookupTable.put(1.25, 1.1d);
        hoodLookupTable.put(1.30, 1.4d);
        hoodLookupTable.put(1.35, 1.8d);
        hoodLookupTable.put(1.40, 2.1d);
        hoodLookupTable.put(1.45, 2.4d);
        hoodLookupTable.put(1.50, 2.7d);
        hoodLookupTable.put(1.55, 3.0d);
        hoodLookupTable.put(1.60, 3.3d);
        hoodLookupTable.put(1.65, 3.6d);
        hoodLookupTable.put(1.70, 3.8d);
        hoodLookupTable.put(1.75, 4.1d);
        hoodLookupTable.put(1.80, 4.8d);
        hoodLookupTable.put(1.85, 4.8d);
        hoodLookupTable.put(1.90, 5.3d);
        hoodLookupTable.put(1.95, 5.4d);
        hoodLookupTable.put(2.00, 5.8d);
        hoodLookupTable.put(2.05, 6.0d);
        hoodLookupTable.put(2.10, 6.2d);
        hoodLookupTable.put(2.15, 6.9d);
        hoodLookupTable.put(2.20, 7.1d);
        hoodLookupTable.put(2.25, 7.3d);
        hoodLookupTable.put(2.30, 8.1d);
        hoodLookupTable.put(2.35, 8.3d);
        hoodLookupTable.put(2.40, 8.4d);
        hoodLookupTable.put(2.45, 9.2d);
        hoodLookupTable.put(2.50, 9.4d);
        hoodLookupTable.put(2.55, 9.5d);
        hoodLookupTable.put(2.60, 10.3d);
        hoodLookupTable.put(2.65, 10.5d);
        hoodLookupTable.put(2.70, 10.6d);
        hoodLookupTable.put(2.75, 11.5d);
        hoodLookupTable.put(2.80, 11.6d);
        hoodLookupTable.put(2.85, 12.3d);
        hoodLookupTable.put(2.90, 12.6d);
        hoodLookupTable.put(2.95, 12.7d);
        hoodLookupTable.put(3.00, 13.5d);
        hoodLookupTable.put(3.05, 13.8d);
        hoodLookupTable.put(3.10, 13.8d);
        hoodLookupTable.put(3.15, 13.9d);
        hoodLookupTable.put(3.20, 15.0d);
        hoodLookupTable.put(3.25, 15.0d);
        hoodLookupTable.put(3.30, 15.0d);
        hoodLookupTable.put(3.35, 16.3d);
        hoodLookupTable.put(3.40, 16.3d);
        hoodLookupTable.put(3.45, 16.3d);
        hoodLookupTable.put(3.50, 16.3d);
        hoodLookupTable.put(3.55, 17.7d);
        hoodLookupTable.put(3.60, 17.7d);
        hoodLookupTable.put(3.65, 17.7d);
        hoodLookupTable.put(3.70, 17.7d);
        hoodLookupTable.put(3.75, 19.2d);
        hoodLookupTable.put(3.80, 19.2d);
        hoodLookupTable.put(3.85, 19.2d);
        hoodLookupTable.put(3.90, 19.2d);
        hoodLookupTable.put(3.95, 20.8d);
        hoodLookupTable.put(4.00, 20.8d);
        hoodLookupTable.put(4.05, 20.8d);
        hoodLookupTable.put(4.10, 20.8d);
        hoodLookupTable.put(4.15, 20.8d);
        hoodLookupTable.put(4.20, 20.8d);
        hoodLookupTable.put(4.25, 22.0d);
        hoodLookupTable.put(4.30, 22.1d);
        hoodLookupTable.put(4.35, 22.1d);
        hoodLookupTable.put(4.40, 22.1d);
        hoodLookupTable.put(4.45, 22.1d);
        hoodLookupTable.put(4.50, 22.1d);
        hoodLookupTable.put(4.55, 22.7d);
        hoodLookupTable.put(4.60, 22.9d);
        hoodLookupTable.put(4.65, 22.9d);
        hoodLookupTable.put(4.70, 22.9d);
        hoodLookupTable.put(4.75, 22.9d);
        hoodLookupTable.put(4.80, 22.9d);
        hoodLookupTable.put(4.85, 22.9d);
        hoodLookupTable.put(4.90, 23.1d);
        hoodLookupTable.put(4.95, 23.1d);
        hoodLookupTable.put(5.00, 23.1d);
        hoodLookupTable.put(5.05, 23.1d);
        hoodLookupTable.put(5.10, 24.0d);
        hoodLookupTable.put(5.15, 24.0d);
        hoodLookupTable.put(5.20, 24.0d);
        hoodLookupTable.put(5.25, 24.0d);
        hoodLookupTable.put(5.30, 24.0d);
        hoodLookupTable.put(5.35, 24.0d);
        hoodLookupTable.put(5.40, 24.0d);
        hoodLookupTable.put(5.45, 24.0d);
        hoodLookupTable.put(5.50, 24.0d);
        hoodLookupTable.put(5.55, 24.0d);
        hoodLookupTable.put(5.60, 24.0d);
        hoodLookupTable.put(5.65, 24.0d);
        hoodLookupTable.put(5.70, 24.0d);
        hoodLookupTable.put(5.75, 24.0d);
        hoodLookupTable.put(5.80, 24.0d);
        hoodLookupTable.put(5.85, 24.4d);
        hoodLookupTable.put(5.90, 24.4d);
        hoodLookupTable.put(5.95, 24.4d);
        hoodLookupTable.put(6.00, 24.4d);
        hoodLookupTable.put(6.05, 24.4d);
        hoodLookupTable.put(6.10, 24.4d);
        hoodLookupTable.put(6.15, 24.7d);
        hoodLookupTable.put(6.20, 24.7d);
        hoodLookupTable.put(6.25, 24.7d);
        hoodLookupTable.put(6.30, 25.0d);
        hoodLookupTable.put(6.35, 25.0d);
        hoodLookupTable.put(6.40, 25.0d);
        hoodLookupTable.put(6.45, 25.0d);
        hoodLookupTable.put(6.50, 25.0d);
        hoodLookupTable.put(6.55, 25.0d);
        hoodLookupTable.put(6.60, 25.0d);
        hoodLookupTable.put(6.65, 25.0d);
        hoodLookupTable.put(6.70, 25.0d);
        hoodLookupTable.put(6.75, 25.0d);
        hoodLookupTable.put(6.80, 25.0d);
        hoodLookupTable.put(6.85, 25.0d);
        hoodLookupTable.put(6.90, 25.0d);
        hoodLookupTable.put(6.95, 25.0d);
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

        shooterSpeedLookupTable.put(1.10, 41.1);
        shooterSpeedLookupTable.put(1.15, 41.4);
        shooterSpeedLookupTable.put(1.20, 41.7);
        shooterSpeedLookupTable.put(1.25, 42.0);
        shooterSpeedLookupTable.put(1.30, 42.3);
        shooterSpeedLookupTable.put(1.35, 42.6);
        shooterSpeedLookupTable.put(1.40, 42.9);
        shooterSpeedLookupTable.put(1.45, 43.2);
        shooterSpeedLookupTable.put(1.50, 43.5);
        shooterSpeedLookupTable.put(1.55, 43.8);
        shooterSpeedLookupTable.put(1.60, 44.0);
        shooterSpeedLookupTable.put(1.65, 44.3);
        shooterSpeedLookupTable.put(1.70, 44.6);
        shooterSpeedLookupTable.put(1.75, 44.9);
        shooterSpeedLookupTable.put(1.80, 44.9);
        shooterSpeedLookupTable.put(1.85, 45.5);
        shooterSpeedLookupTable.put(1.90, 45.5);
        shooterSpeedLookupTable.put(1.95, 45.9);
        shooterSpeedLookupTable.put(2.00, 46.1);
        shooterSpeedLookupTable.put(2.05, 46.4);
        shooterSpeedLookupTable.put(2.10, 46.7);
        shooterSpeedLookupTable.put(2.15, 46.7);
        shooterSpeedLookupTable.put(2.20, 47.0);
        shooterSpeedLookupTable.put(2.25, 47.3);
        shooterSpeedLookupTable.put(2.30, 47.3);
        shooterSpeedLookupTable.put(2.35, 47.6);
        shooterSpeedLookupTable.put(2.40, 47.9);
        shooterSpeedLookupTable.put(2.45, 47.9);
        shooterSpeedLookupTable.put(2.50, 48.1);
        shooterSpeedLookupTable.put(2.55, 48.4);
        shooterSpeedLookupTable.put(2.60, 48.4);
        shooterSpeedLookupTable.put(2.65, 48.7);
        shooterSpeedLookupTable.put(2.70, 49.0);
        shooterSpeedLookupTable.put(2.75, 49.0);
        shooterSpeedLookupTable.put(2.80, 49.3);
        shooterSpeedLookupTable.put(2.85, 49.5);
        shooterSpeedLookupTable.put(2.90, 49.6);
        shooterSpeedLookupTable.put(2.95, 49.9);
        shooterSpeedLookupTable.put(3.00, 50.0);
        shooterSpeedLookupTable.put(3.05, 50.2);
        shooterSpeedLookupTable.put(3.10, 50.5);
        shooterSpeedLookupTable.put(3.15, 50.8);
        shooterSpeedLookupTable.put(3.20, 50.8);
        shooterSpeedLookupTable.put(3.25, 51.1);
        shooterSpeedLookupTable.put(3.30, 51.4);
        shooterSpeedLookupTable.put(3.35, 51.4);
        shooterSpeedLookupTable.put(3.40, 51.7);
        shooterSpeedLookupTable.put(3.45, 52.0);
        shooterSpeedLookupTable.put(3.50, 52.2);
        shooterSpeedLookupTable.put(3.55, 52.2);
        shooterSpeedLookupTable.put(3.60, 52.5);
        shooterSpeedLookupTable.put(3.65, 52.8);
        shooterSpeedLookupTable.put(3.70, 53.1);
        shooterSpeedLookupTable.put(3.75, 53.1);
        shooterSpeedLookupTable.put(3.80, 53.4);
        shooterSpeedLookupTable.put(3.85, 53.7);
        shooterSpeedLookupTable.put(3.90, 54.0);
        shooterSpeedLookupTable.put(3.95, 54.2);
        shooterSpeedLookupTable.put(4.00, 54.3);
        shooterSpeedLookupTable.put(4.05, 54.6);
        shooterSpeedLookupTable.put(4.10, 54.9);
        shooterSpeedLookupTable.put(4.15, 55.2);
        shooterSpeedLookupTable.put(4.20, 55.5);
        shooterSpeedLookupTable.put(4.25, 55.6);
        shooterSpeedLookupTable.put(4.30, 55.8);
        shooterSpeedLookupTable.put(4.35, 56.1);
        shooterSpeedLookupTable.put(4.40, 56.3);
        shooterSpeedLookupTable.put(4.45, 56.6);
        shooterSpeedLookupTable.put(4.50, 56.9);
        shooterSpeedLookupTable.put(4.55, 57.1);
        shooterSpeedLookupTable.put(4.60, 57.2);
        shooterSpeedLookupTable.put(4.65, 57.5);
        shooterSpeedLookupTable.put(4.70, 57.8);
        shooterSpeedLookupTable.put(4.75, 58.1);
        shooterSpeedLookupTable.put(4.80, 58.4);
        shooterSpeedLookupTable.put(4.85, 58.5);
        shooterSpeedLookupTable.put(4.90, 58.7);
        shooterSpeedLookupTable.put(4.95, 59.0);
        shooterSpeedLookupTable.put(5.00, 59.3);
        shooterSpeedLookupTable.put(5.05, 59.6);
        shooterSpeedLookupTable.put(5.10, 59.7);
        shooterSpeedLookupTable.put(5.15, 60.0);
        shooterSpeedLookupTable.put(5.20, 60.2);
        shooterSpeedLookupTable.put(5.25, 60.4);
        shooterSpeedLookupTable.put(5.30, 60.7);
        shooterSpeedLookupTable.put(5.35, 61.0);
        shooterSpeedLookupTable.put(5.40, 61.2);
        shooterSpeedLookupTable.put(5.45, 61.3);
        shooterSpeedLookupTable.put(5.50, 61.6);
        shooterSpeedLookupTable.put(5.55, 61.9);
        shooterSpeedLookupTable.put(5.60, 62.2);
        shooterSpeedLookupTable.put(5.65, 62.4);
        shooterSpeedLookupTable.put(5.70, 62.5);
        shooterSpeedLookupTable.put(5.75, 62.8);
        shooterSpeedLookupTable.put(5.80, 63.1);
        shooterSpeedLookupTable.put(5.85, 63.2);
        shooterSpeedLookupTable.put(5.90, 63.5);
        shooterSpeedLookupTable.put(5.95, 63.7);
        shooterSpeedLookupTable.put(6.00, 64.0);
        shooterSpeedLookupTable.put(6.05, 64.3);
        shooterSpeedLookupTable.put(6.10, 64.4);
        shooterSpeedLookupTable.put(6.15, 64.6);
        shooterSpeedLookupTable.put(6.20, 64.8);
        shooterSpeedLookupTable.put(6.25, 65.1);
        shooterSpeedLookupTable.put(6.30, 65.3);
        shooterSpeedLookupTable.put(6.35, 65.6);
        shooterSpeedLookupTable.put(6.40, 65.7);
        shooterSpeedLookupTable.put(6.45, 66.0);
        shooterSpeedLookupTable.put(6.50, 66.3);
        shooterSpeedLookupTable.put(6.55, 66.5);
        shooterSpeedLookupTable.put(6.60, 66.6);
        shooterSpeedLookupTable.put(6.65, 66.9);
        shooterSpeedLookupTable.put(6.70, 67.2);
        shooterSpeedLookupTable.put(6.75, 67.3);
        shooterSpeedLookupTable.put(6.80, 67.6);
        shooterSpeedLookupTable.put(6.85, 67.8);
        shooterSpeedLookupTable.put(6.90, 68.1);
        shooterSpeedLookupTable.put(6.95, 68.4);
        shooterSpeedLookupTable.put(7.00, 68.5);
        shooterSpeedLookupTable.put(7.05, 68.7);
        shooterSpeedLookupTable.put(7.10, 68.9);
        shooterSpeedLookupTable.put(7.15, 69.2);
        shooterSpeedLookupTable.put(7.20, 69.4);
        shooterSpeedLookupTable.put(7.25, 69.5);
        shooterSpeedLookupTable.put(7.30, 69.8);
        shooterSpeedLookupTable.put(7.35, 70.1);
        shooterSpeedLookupTable.put(7.40, 70.3);
        shooterSpeedLookupTable.put(7.45, 70.6);
        shooterSpeedLookupTable.put(7.50, 70.7);
        shooterSpeedLookupTable.put(7.55, 71.0);
        shooterSpeedLookupTable.put(7.60, 71.1);
        shooterSpeedLookupTable.put(7.65, 71.4);
        shooterSpeedLookupTable.put(7.70, 71.6);
        shooterSpeedLookupTable.put(7.75, 71.9);
        shooterSpeedLookupTable.put(7.80, 72.0);
        shooterSpeedLookupTable.put(7.85, 72.3);
        shooterSpeedLookupTable.put(7.90, 72.5);
        shooterSpeedLookupTable.put(7.95, 72.8);
        shooterSpeedLookupTable.put(8.00, 72.9);

        timeOfFlightLookupTable.put(1.10, 0.766);
        timeOfFlightLookupTable.put(1.15, 0.776);
        timeOfFlightLookupTable.put(1.20, 0.790);
        timeOfFlightLookupTable.put(1.25, 0.798);
        timeOfFlightLookupTable.put(1.30, 0.810);
        timeOfFlightLookupTable.put(1.35, 0.816);
        timeOfFlightLookupTable.put(1.40, 0.827);
        timeOfFlightLookupTable.put(1.45, 0.837);
        timeOfFlightLookupTable.put(1.50, 0.846);
        timeOfFlightLookupTable.put(1.55, 0.855);
        timeOfFlightLookupTable.put(1.60, 0.864);
        timeOfFlightLookupTable.put(1.65, 0.872);
        timeOfFlightLookupTable.put(1.70, 0.884);
        timeOfFlightLookupTable.put(1.75, 0.891);
        timeOfFlightLookupTable.put(1.80, 0.885);
        timeOfFlightLookupTable.put(1.85, 0.899);
        timeOfFlightLookupTable.put(1.90, 0.902);
        timeOfFlightLookupTable.put(1.95, 0.913);
        timeOfFlightLookupTable.put(2.00, 0.917);
        timeOfFlightLookupTable.put(2.05, 0.926);
        timeOfFlightLookupTable.put(2.10, 0.935);
        timeOfFlightLookupTable.put(2.15, 0.928);
        timeOfFlightLookupTable.put(2.20, 0.936);
        timeOfFlightLookupTable.put(2.25, 0.944);
        timeOfFlightLookupTable.put(2.30, 0.933);
        timeOfFlightLookupTable.put(2.35, 0.941);
        timeOfFlightLookupTable.put(2.40, 0.952);
        timeOfFlightLookupTable.put(2.45, 0.941);
        timeOfFlightLookupTable.put(2.50, 0.948);
        timeOfFlightLookupTable.put(2.55, 0.958);
        timeOfFlightLookupTable.put(2.60, 0.948);
        timeOfFlightLookupTable.put(2.65, 0.954);
        timeOfFlightLookupTable.put(2.70, 0.963);
        timeOfFlightLookupTable.put(2.75, 0.950);
        timeOfFlightLookupTable.put(2.80, 0.959);
        timeOfFlightLookupTable.put(2.85, 0.950);
        timeOfFlightLookupTable.put(2.90, 0.955);
        timeOfFlightLookupTable.put(2.95, 0.963);
        timeOfFlightLookupTable.put(3.00, 0.951);
        timeOfFlightLookupTable.put(3.05, 0.956);
        timeOfFlightLookupTable.put(3.10, 0.967);
        timeOfFlightLookupTable.put(3.15, 0.974);
        timeOfFlightLookupTable.put(3.20, 0.956);
        timeOfFlightLookupTable.put(3.25, 0.966);
        timeOfFlightLookupTable.put(3.30, 0.976);
        timeOfFlightLookupTable.put(3.35, 0.954);
        timeOfFlightLookupTable.put(3.40, 0.963);
        timeOfFlightLookupTable.put(3.45, 0.973);
        timeOfFlightLookupTable.put(3.50, 0.982);
        timeOfFlightLookupTable.put(3.55, 0.957);
        timeOfFlightLookupTable.put(3.60, 0.966);
        timeOfFlightLookupTable.put(3.65, 0.975);
        timeOfFlightLookupTable.put(3.70, 0.984);
        timeOfFlightLookupTable.put(3.75, 0.958);
        timeOfFlightLookupTable.put(3.80, 0.966);
        timeOfFlightLookupTable.put(3.85, 0.974);
        timeOfFlightLookupTable.put(3.90, 0.982);
        timeOfFlightLookupTable.put(3.95, 0.953);
        timeOfFlightLookupTable.put(4.00, 0.963);
        timeOfFlightLookupTable.put(4.05, 0.970);
        timeOfFlightLookupTable.put(4.10, 0.978);
        timeOfFlightLookupTable.put(4.15, 0.985);
        timeOfFlightLookupTable.put(4.20, 0.993);
        timeOfFlightLookupTable.put(4.25, 0.973);
        timeOfFlightLookupTable.put(4.30, 0.981);
        timeOfFlightLookupTable.put(4.35, 0.988);
        timeOfFlightLookupTable.put(4.40, 0.994);
        timeOfFlightLookupTable.put(4.45, 1.001);
        timeOfFlightLookupTable.put(4.50, 1.008);
        timeOfFlightLookupTable.put(4.55, 1.003);
        timeOfFlightLookupTable.put(4.60, 1.008);
        timeOfFlightLookupTable.put(4.65, 1.014);
        timeOfFlightLookupTable.put(4.70, 1.021);
        timeOfFlightLookupTable.put(4.75, 1.027);
        timeOfFlightLookupTable.put(4.80, 1.033);
        timeOfFlightLookupTable.put(4.85, 1.042);
        timeOfFlightLookupTable.put(4.90, 1.046);
        timeOfFlightLookupTable.put(4.95, 1.053);
        timeOfFlightLookupTable.put(5.00, 1.059);
        timeOfFlightLookupTable.put(5.05, 1.065);
        timeOfFlightLookupTable.put(5.10, 1.052);
        timeOfFlightLookupTable.put(5.15, 1.058);
        timeOfFlightLookupTable.put(5.20, 1.066);
        timeOfFlightLookupTable.put(5.25, 1.072);
        timeOfFlightLookupTable.put(5.30, 1.078);
        timeOfFlightLookupTable.put(5.35, 1.084);
        timeOfFlightLookupTable.put(5.40, 1.092);
        timeOfFlightLookupTable.put(5.45, 1.100);
        timeOfFlightLookupTable.put(5.50, 1.106);
        timeOfFlightLookupTable.put(5.55, 1.111);
        timeOfFlightLookupTable.put(5.60, 1.117);
        timeOfFlightLookupTable.put(5.65, 1.125);
        timeOfFlightLookupTable.put(5.70, 1.133);
        timeOfFlightLookupTable.put(5.75, 1.139);
        timeOfFlightLookupTable.put(5.80, 1.144);
        timeOfFlightLookupTable.put(5.85, 1.142);
        timeOfFlightLookupTable.put(5.90, 1.147);
        timeOfFlightLookupTable.put(5.95, 1.155);
        timeOfFlightLookupTable.put(6.00, 1.160);
        timeOfFlightLookupTable.put(6.05, 1.166);
        timeOfFlightLookupTable.put(6.10, 1.173);
        timeOfFlightLookupTable.put(6.15, 1.173);
        timeOfFlightLookupTable.put(6.20, 1.178);
        timeOfFlightLookupTable.put(6.25, 1.184);
        timeOfFlightLookupTable.put(6.30, 1.183);
        timeOfFlightLookupTable.put(6.35, 1.188);
        timeOfFlightLookupTable.put(6.40, 1.196);
        timeOfFlightLookupTable.put(6.45, 1.201);
        timeOfFlightLookupTable.put(6.50, 1.206);
        timeOfFlightLookupTable.put(6.55, 1.213);
        timeOfFlightLookupTable.put(6.60, 1.221);
        timeOfFlightLookupTable.put(6.65, 1.225);
        timeOfFlightLookupTable.put(6.70, 1.230);
        timeOfFlightLookupTable.put(6.75, 1.238);
        timeOfFlightLookupTable.put(6.80, 1.242);
        timeOfFlightLookupTable.put(6.85, 1.250);
        timeOfFlightLookupTable.put(6.90, 1.254);
        timeOfFlightLookupTable.put(6.95, 1.259);
        timeOfFlightLookupTable.put(7.00, 1.266);
        timeOfFlightLookupTable.put(7.05, 1.273);
        timeOfFlightLookupTable.put(7.10, 1.278);
        timeOfFlightLookupTable.put(7.15, 1.283);
        timeOfFlightLookupTable.put(7.20, 1.290);
        timeOfFlightLookupTable.put(7.25, 1.297);
        timeOfFlightLookupTable.put(7.30, 1.301);
        timeOfFlightLookupTable.put(7.35, 1.306);
        timeOfFlightLookupTable.put(7.40, 1.313);
        timeOfFlightLookupTable.put(7.45, 1.317);
        timeOfFlightLookupTable.put(7.50, 1.324);
        timeOfFlightLookupTable.put(7.55, 1.329);
        timeOfFlightLookupTable.put(7.60, 1.336);
        timeOfFlightLookupTable.put(7.65, 1.340);
        timeOfFlightLookupTable.put(7.70, 1.347);
        timeOfFlightLookupTable.put(7.75, 1.351);
        timeOfFlightLookupTable.put(7.80, 1.358);
        timeOfFlightLookupTable.put(7.85, 1.363);
        timeOfFlightLookupTable.put(7.90, 1.369);
        timeOfFlightLookupTable.put(7.95, 1.374);
        timeOfFlightLookupTable.put(8.00, 1.381);

        //Shuttling Look up Tables

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
//            if (futureTurretToTargetDistance >= 4.5) {
//                if ((Robot.getAlliance().equals(Alliance.RED)
//                        && futureTurretPosition.getY() < Constants.RED_HUB_CENTER.getY())
//                        || (Robot.getAlliance().equals(Alliance.BLUE)
//                        && futureTurretPosition.getY() > Constants.BLUE_HUB_CENTER.getY())) {
//                    turretAngle += 1d;
//                    turretAnglePhaseDelayed += 1d;
//                    shooterSpeed += .75;
//                    shooterSpeedPhaseDelayed += .75;
//                }
//            }
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
        return futureTurretToTargetDistance >= 1.1 && futureTurretToTargetDistance <= 8d;
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