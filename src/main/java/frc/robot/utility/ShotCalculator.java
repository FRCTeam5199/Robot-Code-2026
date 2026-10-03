package frc.robot.utility;

import frc.robot.Robot;
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
import org.wpilib.math.util.Pair;

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
        hoodLookupTable.put(1.15, 0.2d);
        hoodLookupTable.put(1.20, 0.6d);
        hoodLookupTable.put(1.25, 0.9d);
        hoodLookupTable.put(1.30, 1.7d);
        hoodLookupTable.put(1.35, 1.7d);
        hoodLookupTable.put(1.40, 2.4d);
        hoodLookupTable.put(1.45, 2.7d);
        hoodLookupTable.put(1.50, 3.0d);
        hoodLookupTable.put(1.55, 3.3d);
        hoodLookupTable.put(1.60, 3.6d);
        hoodLookupTable.put(1.65, 3.9d);
        hoodLookupTable.put(1.70, 4.2d);
        hoodLookupTable.put(1.75, 4.5d);
        hoodLookupTable.put(1.80, 4.7d);
        hoodLookupTable.put(1.85, 5.0d);
        hoodLookupTable.put(1.90, 5.2d);
        hoodLookupTable.put(1.95, 5.5d);
        hoodLookupTable.put(2.00, 5.7d);
        hoodLookupTable.put(2.05, 5.9d);
        hoodLookupTable.put(2.10, 6.6d);
        hoodLookupTable.put(2.15, 6.8d);
        hoodLookupTable.put(2.20, 7.0d);
        hoodLookupTable.put(2.25, 7.8d);
        hoodLookupTable.put(2.30, 8.0d);
        hoodLookupTable.put(2.35, 8.2d);
        hoodLookupTable.put(2.40, 8.8d);
        hoodLookupTable.put(2.45, 9.1d);
        hoodLookupTable.put(2.50, 9.3d);
        hoodLookupTable.put(2.55, 10.1d);
        hoodLookupTable.put(2.60, 10.3d);
        hoodLookupTable.put(2.65, 10.4d);
        hoodLookupTable.put(2.70, 11.3d);
        hoodLookupTable.put(2.75, 11.4d);
        hoodLookupTable.put(2.80, 11.5d);
        hoodLookupTable.put(2.85, 12.3d);
        hoodLookupTable.put(2.90, 12.6d);
        hoodLookupTable.put(2.95, 12.7d);
        hoodLookupTable.put(3.00, 13.5d);
        hoodLookupTable.put(3.05, 13.8d);
        hoodLookupTable.put(3.10, 13.8d);
        hoodLookupTable.put(3.15, 13.9d);
        hoodLookupTable.put(3.20, 15.0d);
        hoodLookupTable.put(3.25, 15.0d);
        hoodLookupTable.put(3.30, 15.1d);
        hoodLookupTable.put(3.35, 16.3d);
        hoodLookupTable.put(3.40, 16.3d);
        hoodLookupTable.put(3.45, 16.3d);
        hoodLookupTable.put(3.50, 16.3d);
        hoodLookupTable.put(3.55, 17.8d);
        hoodLookupTable.put(3.60, 17.8d);
        hoodLookupTable.put(3.65, 17.8d);
        hoodLookupTable.put(3.70, 17.8d);
        hoodLookupTable.put(3.75, 19.1d);
        hoodLookupTable.put(3.80, 19.2d);
        hoodLookupTable.put(3.85, 19.2d);
        hoodLookupTable.put(3.90, 19.2d);
        hoodLookupTable.put(3.95, 19.2d);
        hoodLookupTable.put(4.00, 20.8d);
        hoodLookupTable.put(4.05, 20.9d);
        hoodLookupTable.put(4.10, 20.9d);
        hoodLookupTable.put(4.15, 20.9d);
        hoodLookupTable.put(4.20, 20.9d);
        hoodLookupTable.put(4.25, 20.9d);
        hoodLookupTable.put(4.30, 20.9d);
        hoodLookupTable.put(4.35, 22.6d);
        hoodLookupTable.put(4.40, 22.6d);
        hoodLookupTable.put(4.45, 22.6d);
        hoodLookupTable.put(4.50, 22.6d);
        hoodLookupTable.put(4.55, 22.6d);
        hoodLookupTable.put(4.60, 22.6d);
        hoodLookupTable.put(4.65, 22.6d);
        hoodLookupTable.put(4.70, 22.6d);
        hoodLookupTable.put(4.75, 22.6d);
        hoodLookupTable.put(4.80, 22.6d);
        hoodLookupTable.put(4.85, 22.6d);
        hoodLookupTable.put(4.90, 22.6d);
        hoodLookupTable.put(4.95, 22.6d);
        hoodLookupTable.put(5.00, 23.0d);
        hoodLookupTable.put(5.05, 23.0d);
        hoodLookupTable.put(5.10, 23.0d);
        hoodLookupTable.put(5.15, 23.0d);
        hoodLookupTable.put(5.20, 23.8d);
        hoodLookupTable.put(5.25, 23.8d);
        hoodLookupTable.put(5.30, 23.8d);
        hoodLookupTable.put(5.35, 23.8d);
        hoodLookupTable.put(5.40, 23.8d);
        hoodLookupTable.put(5.45, 23.8d);
        hoodLookupTable.put(5.50, 23.8d);
        hoodLookupTable.put(5.55, 23.8d);
        hoodLookupTable.put(5.60, 23.8d);
        hoodLookupTable.put(5.65, 23.8d);
        hoodLookupTable.put(5.70, 23.8d);
        hoodLookupTable.put(5.75, 23.8d);
        hoodLookupTable.put(5.80, 24.1d);
        hoodLookupTable.put(5.85, 24.1d);
        hoodLookupTable.put(5.90, 24.1d);
        hoodLookupTable.put(5.95, 24.1d);
        hoodLookupTable.put(6.00, 24.1d);
        hoodLookupTable.put(6.05, 24.3d);
        hoodLookupTable.put(6.10, 24.3d);
        hoodLookupTable.put(6.15, 24.3d);
        hoodLookupTable.put(6.20, 24.9d);
        hoodLookupTable.put(6.25, 24.9d);
        hoodLookupTable.put(6.30, 24.9d);
        hoodLookupTable.put(6.35, 24.9d);
        hoodLookupTable.put(6.40, 24.9d);
        hoodLookupTable.put(6.45, 24.9d);
        hoodLookupTable.put(6.50, 24.9d);
        hoodLookupTable.put(6.55, 24.9d);
        hoodLookupTable.put(6.60, 24.9d);
        hoodLookupTable.put(6.65, 24.9d);
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

        shooterSpeedLookupTable.put(1.10, 41.7);
        shooterSpeedLookupTable.put(1.15, 42.0);
        shooterSpeedLookupTable.put(1.20, 42.3);
        shooterSpeedLookupTable.put(1.25, 42.6);
        shooterSpeedLookupTable.put(1.30, 42.6);
        shooterSpeedLookupTable.put(1.35, 43.2);
        shooterSpeedLookupTable.put(1.40, 43.2);
        shooterSpeedLookupTable.put(1.45, 43.5);
        shooterSpeedLookupTable.put(1.50, 43.8);
        shooterSpeedLookupTable.put(1.55, 44.0);
        shooterSpeedLookupTable.put(1.60, 44.3);
        shooterSpeedLookupTable.put(1.65, 44.6);
        shooterSpeedLookupTable.put(1.70, 44.9);
        shooterSpeedLookupTable.put(1.75, 45.2);
        shooterSpeedLookupTable.put(1.80, 45.5);
        shooterSpeedLookupTable.put(1.85, 45.8);
        shooterSpeedLookupTable.put(1.90, 46.1);
        shooterSpeedLookupTable.put(1.95, 46.4);
        shooterSpeedLookupTable.put(2.00, 46.7);
        shooterSpeedLookupTable.put(2.05, 47.0);
        shooterSpeedLookupTable.put(2.10, 47.0);
        shooterSpeedLookupTable.put(2.15, 47.3);
        shooterSpeedLookupTable.put(2.20, 47.6);
        shooterSpeedLookupTable.put(2.25, 47.6);
        shooterSpeedLookupTable.put(2.30, 47.9);
        shooterSpeedLookupTable.put(2.35, 48.1);
        shooterSpeedLookupTable.put(2.40, 48.3);
        shooterSpeedLookupTable.put(2.45, 48.4);
        shooterSpeedLookupTable.put(2.50, 48.7);
        shooterSpeedLookupTable.put(2.55, 48.7);
        shooterSpeedLookupTable.put(2.60, 49.0);
        shooterSpeedLookupTable.put(2.65, 49.3);
        shooterSpeedLookupTable.put(2.70, 49.3);
        shooterSpeedLookupTable.put(2.75, 49.6);
        shooterSpeedLookupTable.put(2.80, 49.9);
        shooterSpeedLookupTable.put(2.85, 50.0);
        shooterSpeedLookupTable.put(2.90, 50.2);
        shooterSpeedLookupTable.put(2.95, 50.5);
        shooterSpeedLookupTable.put(3.00, 50.6);
        shooterSpeedLookupTable.put(3.05, 50.8);
        shooterSpeedLookupTable.put(3.10, 51.1);
        shooterSpeedLookupTable.put(3.15, 51.4);
        shooterSpeedLookupTable.put(3.20, 51.4);
        shooterSpeedLookupTable.put(3.25, 51.7);
        shooterSpeedLookupTable.put(3.30, 52.0);
        shooterSpeedLookupTable.put(3.35, 52.0);
        shooterSpeedLookupTable.put(3.40, 52.2);
        shooterSpeedLookupTable.put(3.45, 52.5);
        shooterSpeedLookupTable.put(3.50, 52.8);
        shooterSpeedLookupTable.put(3.55, 52.8);
        shooterSpeedLookupTable.put(3.60, 53.1);
        shooterSpeedLookupTable.put(3.65, 53.4);
        shooterSpeedLookupTable.put(3.70, 53.7);
        shooterSpeedLookupTable.put(3.75, 53.9);
        shooterSpeedLookupTable.put(3.80, 54.0);
        shooterSpeedLookupTable.put(3.85, 54.3);
        shooterSpeedLookupTable.put(3.90, 54.6);
        shooterSpeedLookupTable.put(3.95, 54.9);
        shooterSpeedLookupTable.put(4.00, 55.0);
        shooterSpeedLookupTable.put(4.05, 55.2);
        shooterSpeedLookupTable.put(4.10, 55.5);
        shooterSpeedLookupTable.put(4.15, 55.8);
        shooterSpeedLookupTable.put(4.20, 56.1);
        shooterSpeedLookupTable.put(4.25, 56.3);
        shooterSpeedLookupTable.put(4.30, 56.6);
        shooterSpeedLookupTable.put(4.35, 56.6);
        shooterSpeedLookupTable.put(4.40, 56.9);
        shooterSpeedLookupTable.put(4.45, 57.2);
        shooterSpeedLookupTable.put(4.50, 57.5);
        shooterSpeedLookupTable.put(4.55, 57.8);
        shooterSpeedLookupTable.put(4.60, 58.1);
        shooterSpeedLookupTable.put(4.65, 58.3);
        shooterSpeedLookupTable.put(4.70, 58.4);
        shooterSpeedLookupTable.put(4.75, 58.7);
        shooterSpeedLookupTable.put(4.80, 59.0);
        shooterSpeedLookupTable.put(4.85, 59.3);
        shooterSpeedLookupTable.put(4.90, 59.6);
        shooterSpeedLookupTable.put(4.95, 59.7);
        shooterSpeedLookupTable.put(5.00, 59.9);
        shooterSpeedLookupTable.put(5.05, 60.2);
        shooterSpeedLookupTable.put(5.10, 60.4);
        shooterSpeedLookupTable.put(5.15, 60.7);
        shooterSpeedLookupTable.put(5.20, 60.9);
        shooterSpeedLookupTable.put(5.25, 61.2);
        shooterSpeedLookupTable.put(5.30, 61.3);
        shooterSpeedLookupTable.put(5.35, 61.6);
        shooterSpeedLookupTable.put(5.40, 61.9);
        shooterSpeedLookupTable.put(5.45, 62.2);
        shooterSpeedLookupTable.put(5.50, 62.4);
        shooterSpeedLookupTable.put(5.55, 62.5);
        shooterSpeedLookupTable.put(5.60, 62.8);
        shooterSpeedLookupTable.put(5.65, 63.1);
        shooterSpeedLookupTable.put(5.70, 63.4);
        shooterSpeedLookupTable.put(5.75, 63.5);
        shooterSpeedLookupTable.put(5.80, 63.7);
        shooterSpeedLookupTable.put(5.85, 64.0);
        shooterSpeedLookupTable.put(5.90, 64.3);
        shooterSpeedLookupTable.put(5.95, 64.6);
        shooterSpeedLookupTable.put(6.00, 64.7);
        shooterSpeedLookupTable.put(6.05, 64.8);
        shooterSpeedLookupTable.put(6.10, 65.1);
        shooterSpeedLookupTable.put(6.15, 65.4);
        shooterSpeedLookupTable.put(6.20, 65.6);
        shooterSpeedLookupTable.put(6.25, 65.9);
        shooterSpeedLookupTable.put(6.30, 66.0);
        shooterSpeedLookupTable.put(6.35, 66.3);
        shooterSpeedLookupTable.put(6.40, 66.6);
        shooterSpeedLookupTable.put(6.45, 66.7);
        shooterSpeedLookupTable.put(6.50, 67.0);
        shooterSpeedLookupTable.put(6.55, 67.2);
        shooterSpeedLookupTable.put(6.60, 67.5);
        shooterSpeedLookupTable.put(6.65, 67.8);
        shooterSpeedLookupTable.put(6.70, 67.9);
        shooterSpeedLookupTable.put(6.75, 68.1);
        shooterSpeedLookupTable.put(6.80, 68.4);
        shooterSpeedLookupTable.put(6.85, 68.7);
        shooterSpeedLookupTable.put(6.90, 68.8);
        shooterSpeedLookupTable.put(6.95, 69.1);
        shooterSpeedLookupTable.put(7.00, 69.2);
        shooterSpeedLookupTable.put(7.05, 69.5);
        shooterSpeedLookupTable.put(7.10, 69.8);
        shooterSpeedLookupTable.put(7.15, 70.0);
        shooterSpeedLookupTable.put(7.20, 70.1);
        shooterSpeedLookupTable.put(7.25, 70.4);
        shooterSpeedLookupTable.put(7.30, 70.7);
        shooterSpeedLookupTable.put(7.35, 70.8);
        shooterSpeedLookupTable.put(7.40, 71.1);
        shooterSpeedLookupTable.put(7.45, 71.3);
        shooterSpeedLookupTable.put(7.50, 71.6);
        shooterSpeedLookupTable.put(7.55, 71.7);
        shooterSpeedLookupTable.put(7.60, 72.0);
        shooterSpeedLookupTable.put(7.65, 72.2);
        shooterSpeedLookupTable.put(7.70, 72.5);
        shooterSpeedLookupTable.put(7.75, 72.8);
        shooterSpeedLookupTable.put(7.80, 72.9);
        shooterSpeedLookupTable.put(7.85, 73.0);
        shooterSpeedLookupTable.put(7.90, 73.3);
        shooterSpeedLookupTable.put(7.95, 73.6);
        shooterSpeedLookupTable.put(8.00, 73.8);

        timeOfFlightLookupTable.put(1.10, 0.764);
        timeOfFlightLookupTable.put(1.15, 0.784);
        timeOfFlightLookupTable.put(1.20, 0.793);
        timeOfFlightLookupTable.put(1.25, 0.806);
        timeOfFlightLookupTable.put(1.30, 0.799);
        timeOfFlightLookupTable.put(1.35, 0.819);
        timeOfFlightLookupTable.put(1.40, 0.816);
        timeOfFlightLookupTable.put(1.45, 0.827);
        timeOfFlightLookupTable.put(1.50, 0.836);
        timeOfFlightLookupTable.put(1.55, 0.845);
        timeOfFlightLookupTable.put(1.60, 0.854);
        timeOfFlightLookupTable.put(1.65, 0.862);
        timeOfFlightLookupTable.put(1.70, 0.870);
        timeOfFlightLookupTable.put(1.75, 0.877);
        timeOfFlightLookupTable.put(1.80, 0.888);
        timeOfFlightLookupTable.put(1.85, 0.894);
        timeOfFlightLookupTable.put(1.90, 0.905);
        timeOfFlightLookupTable.put(1.95, 0.910);
        timeOfFlightLookupTable.put(2.00, 0.920);
        timeOfFlightLookupTable.put(2.05, 0.929);
        timeOfFlightLookupTable.put(2.10, 0.922);
        timeOfFlightLookupTable.put(2.15, 0.931);
        timeOfFlightLookupTable.put(2.20, 0.939);
        timeOfFlightLookupTable.put(2.25, 0.929);
        timeOfFlightLookupTable.put(2.30, 0.936);
        timeOfFlightLookupTable.put(2.35, 0.944);
        timeOfFlightLookupTable.put(2.40, 0.938);
        timeOfFlightLookupTable.put(2.45, 0.944);
        timeOfFlightLookupTable.put(2.50, 0.951);
        timeOfFlightLookupTable.put(2.55, 0.941);
        timeOfFlightLookupTable.put(2.60, 0.947);
        timeOfFlightLookupTable.put(2.65, 0.957);
        timeOfFlightLookupTable.put(2.70, 0.944);
        timeOfFlightLookupTable.put(2.75, 0.953);
        timeOfFlightLookupTable.put(2.80, 0.962);
        timeOfFlightLookupTable.put(2.85, 0.950);
        timeOfFlightLookupTable.put(2.90, 0.955);
        timeOfFlightLookupTable.put(2.95, 0.963);
        timeOfFlightLookupTable.put(3.00, 0.951);
        timeOfFlightLookupTable.put(3.05, 0.956);
        timeOfFlightLookupTable.put(3.10, 0.966);
        timeOfFlightLookupTable.put(3.15, 0.974);
        timeOfFlightLookupTable.put(3.20, 0.956);
        timeOfFlightLookupTable.put(3.25, 0.966);
        timeOfFlightLookupTable.put(3.30, 0.973);
        timeOfFlightLookupTable.put(3.35, 0.954);
        timeOfFlightLookupTable.put(3.40, 0.963);
        timeOfFlightLookupTable.put(3.45, 0.973);
        timeOfFlightLookupTable.put(3.50, 0.982);
        timeOfFlightLookupTable.put(3.55, 0.955);
        timeOfFlightLookupTable.put(3.60, 0.964);
        timeOfFlightLookupTable.put(3.65, 0.973);
        timeOfFlightLookupTable.put(3.70, 0.981);
        timeOfFlightLookupTable.put(3.75, 0.958);
        timeOfFlightLookupTable.put(3.80, 0.966);
        timeOfFlightLookupTable.put(3.85, 0.974);
        timeOfFlightLookupTable.put(3.90, 0.983);
        timeOfFlightLookupTable.put(3.95, 0.991);
        timeOfFlightLookupTable.put(4.00, 0.961);
        timeOfFlightLookupTable.put(4.05, 0.969);
        timeOfFlightLookupTable.put(4.10, 0.976);
        timeOfFlightLookupTable.put(4.15, 0.984);
        timeOfFlightLookupTable.put(4.20, 0.991);
        timeOfFlightLookupTable.put(4.25, 0.998);
        timeOfFlightLookupTable.put(4.30, 1.006);
        timeOfFlightLookupTable.put(4.35, 0.977);
        timeOfFlightLookupTable.put(4.40, 0.984);
        timeOfFlightLookupTable.put(4.45, 0.991);
        timeOfFlightLookupTable.put(4.50, 0.997);
        timeOfFlightLookupTable.put(4.55, 1.004);
        timeOfFlightLookupTable.put(4.60, 1.011);
        timeOfFlightLookupTable.put(4.65, 1.020);
        timeOfFlightLookupTable.put(4.70, 1.029);
        timeOfFlightLookupTable.put(4.75, 1.036);
        timeOfFlightLookupTable.put(4.80, 1.042);
        timeOfFlightLookupTable.put(4.85, 1.049);
        timeOfFlightLookupTable.put(4.90, 1.055);
        timeOfFlightLookupTable.put(4.95, 1.064);
        timeOfFlightLookupTable.put(5.00, 1.063);
        timeOfFlightLookupTable.put(5.05, 1.069);
        timeOfFlightLookupTable.put(5.10, 1.075);
        timeOfFlightLookupTable.put(5.15, 1.081);
        timeOfFlightLookupTable.put(5.20, 1.070);
        timeOfFlightLookupTable.put(5.25, 1.076);
        timeOfFlightLookupTable.put(5.30, 1.085);
        timeOfFlightLookupTable.put(5.35, 1.090);
        timeOfFlightLookupTable.put(5.40, 1.096);
        timeOfFlightLookupTable.put(5.45, 1.102);
        timeOfFlightLookupTable.put(5.50, 1.110);
        timeOfFlightLookupTable.put(5.55, 1.119);
        timeOfFlightLookupTable.put(5.60, 1.124);
        timeOfFlightLookupTable.put(5.65, 1.130);
        timeOfFlightLookupTable.put(5.70, 1.136);
        timeOfFlightLookupTable.put(5.75, 1.144);
        timeOfFlightLookupTable.put(5.80, 1.144);
        timeOfFlightLookupTable.put(5.85, 1.149);
        timeOfFlightLookupTable.put(5.90, 1.155);
        timeOfFlightLookupTable.put(5.95, 1.160);
        timeOfFlightLookupTable.put(6.00, 1.168);
        timeOfFlightLookupTable.put(6.05, 1.171);
        timeOfFlightLookupTable.put(6.10, 1.176);
        timeOfFlightLookupTable.put(6.15, 1.181);
        timeOfFlightLookupTable.put(6.20, 1.174);
        timeOfFlightLookupTable.put(6.25, 1.179);
        timeOfFlightLookupTable.put(6.30, 1.186);
        timeOfFlightLookupTable.put(6.35, 1.191);
        timeOfFlightLookupTable.put(6.40, 1.196);
        timeOfFlightLookupTable.put(6.45, 1.204);
        timeOfFlightLookupTable.put(6.50, 1.209);
        timeOfFlightLookupTable.put(6.55, 1.216);
        timeOfFlightLookupTable.put(6.60, 1.221);
        timeOfFlightLookupTable.put(6.65, 1.226);
        timeOfFlightLookupTable.put(6.70, 1.234);
        timeOfFlightLookupTable.put(6.75, 1.241);
        timeOfFlightLookupTable.put(6.80, 1.246);
        timeOfFlightLookupTable.put(6.85, 1.251);
        timeOfFlightLookupTable.put(6.90, 1.258);
        timeOfFlightLookupTable.put(6.95, 1.263);
        timeOfFlightLookupTable.put(7.00, 1.270);
        timeOfFlightLookupTable.put(7.05, 1.275);
        timeOfFlightLookupTable.put(7.10, 1.279);
        timeOfFlightLookupTable.put(7.15, 1.287);
        timeOfFlightLookupTable.put(7.20, 1.294);
        timeOfFlightLookupTable.put(7.25, 1.298);
        timeOfFlightLookupTable.put(7.30, 1.303);
        timeOfFlightLookupTable.put(7.35, 1.310);
        timeOfFlightLookupTable.put(7.40, 1.315);
        timeOfFlightLookupTable.put(7.45, 1.322);
        timeOfFlightLookupTable.put(7.50, 1.326);
        timeOfFlightLookupTable.put(7.55, 1.333);
        timeOfFlightLookupTable.put(7.60, 1.338);
        timeOfFlightLookupTable.put(7.65, 1.345);
        timeOfFlightLookupTable.put(7.70, 1.349);
        timeOfFlightLookupTable.put(7.75, 1.353);
        timeOfFlightLookupTable.put(7.80, 1.360);
        timeOfFlightLookupTable.put(7.85, 1.367);
        timeOfFlightLookupTable.put(7.90, 1.372);
        timeOfFlightLookupTable.put(7.95, 1.376);
        timeOfFlightLookupTable.put(8.00, 1.383);

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
//            Pose2d estimatedPosePhaseDelayed =
//                    estimatedPose.plus(
//                            new Twist2d(
//                                    Math.pow(RobotContainer.getVelocity().vx, 2) * Constants.PHASE_DELAY
//                                            + RobotContainer.getAccelerationX() * Constants.ACCELERATION_PHASE_DELAY,
//                                    Math.pow(RobotContainer.getVelocity().vy, 2) * Constants.PHASE_DELAY
//                                            + RobotContainer.getAccelerationY() * Constants.ACCELERATION_PHASE_DELAY,
//                                    Math.pow(RobotContainer.getVelocity().omega, 2) * Constants.ROTATIONAL_PHASE_DELAY
//                                            + RobotContainer.getAccelerationOmega() * Constants.ACCELERATION_PHASE_DELAY).exp());

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

            Translation2d deltaPhaseDelayed = target.minus(futureTurretPositionPhaseDelayed.getTranslation());
            turretRotationPhaseDelayed = (deltaPhaseDelayed.getX() == 0 && deltaPhaseDelayed.getY() == 0)
                    ? lastTurretRotationPhaseDelayed
                    : new Rotation2d(deltaPhaseDelayed.getX(), deltaPhaseDelayed.getY());

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
            Translation2d delta = target.minus(futureTurretPosition.getTranslation());
            turretRotation = (delta.getX() == 0 && delta.getY() == 0)
                    ? lastTurretRotation
                    : new Rotation2d(delta.getX(), delta.getY());

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