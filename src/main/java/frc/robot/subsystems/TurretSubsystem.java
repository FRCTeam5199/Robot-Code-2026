package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import frc.robot.constants.TurretConstants;
import frc.robot.subsystems.templates.TemplateSubsystem;
import frc.robot.utility.ShotCalculator;
import frc.robot.utility.Type;

public class TurretSubsystem extends TemplateSubsystem {
    private static TurretSubsystem turretSubsystem;

    private TrapezoidProfile profile;
    private TrapezoidProfile.State currentState;
    private TrapezoidProfile.State goalState;

    private double goalRotations;

    private boolean stopMoving = false;

    private NetworkTable networkTable;
    private DoublePublisher goalPositionLogging;
    private DoublePublisher currentPositionLogging;
    private DoublePublisher goalVelocityLogging;
    private DoublePublisher currentVelocityLogging;
    private DoublePublisher turretToTargetDistance;
    private StructPublisher<Pose2d> turretPose;

    private TurretSubsystem() {
        super(Type.ROLLER, TurretConstants.TURRET_MOTOR_ID,
                TurretConstants.TURRET_VELOCITY, TurretConstants.TURRET_ACCELERATION,
                TurretConstants.TURRET_JERK,
                TurretConstants.TURRET_LOWER_TOLERANCE,
                TurretConstants.TURRET_UPPER_TOLERANCE,
                TurretConstants.TURRET_GEAR_RATIO, "Turret");

        configureMotor(TurretConstants.TURRET_INVERTED, TurretConstants.TURRET_BRAKE,
                TurretConstants.TURRET_SUPPLY_CURRENT_LIMIT,
                TurretConstants.TURRET_STATOR_CURRENT_LIMIT,
                TurretConstants.TURRET_SLOT0_CONFIGS);

        configureSometimesEncoder(TurretConstants.TURRET_ENCODER_ID,
                "rio", TurretConstants.TURRET_ENCODER_MAGNET_OFFSET,
                TurretConstants.TURRET_SENSOR_TO_MECH_GEAR_RATIO,
                TurretConstants.TURRET_MOTOR_TO_SENSOR_GEAR_RATIO,
                TurretConstants.TURRET_CCW_POSITIVE, TurretConstants.TURRET_ABSOLUTE_DISCONTINUITY_POINT);

        configureRoller(TurretConstants.TURRET_MIN, TurretConstants.TURRET_MAX);

        configureCustomFF();

        profile = new TrapezoidProfile(new TrapezoidProfile
                .Constraints(TurretConstants.TURRET_VELOCITY, TurretConstants.TURRET_ACCELERATION));
        currentState = new TrapezoidProfile.State(0, 0);
        goalState = new TrapezoidProfile.State(0, 0);

        networkTable = NetworkTableInstance.getDefault().getTable("AutoTracking/");

        goalPositionLogging = networkTable.getDoubleTopic("Goal Position").publish();
        currentPositionLogging = networkTable.getDoubleTopic("Current Position").publish();
        goalVelocityLogging = networkTable.getDoubleTopic("Goal Velocity").publish();
        currentVelocityLogging = networkTable.getDoubleTopic("Current Velocity").publish();
        turretToTargetDistance = networkTable.getDoubleTopic("Distance").publish();
        turretPose = networkTable.getStructTopic("Turret Pose", Pose2d.struct).publish();
    }

    public static TurretSubsystem getInstance() {
        if (turretSubsystem == null) {
            turretSubsystem = new TurretSubsystem();
        }
        return turretSubsystem;
    }

    @Override
    public void periodic() {
        super.periodic();

        goalPositionLogging.set(currentState.position);
        currentPositionLogging.set(getMotorRot());

        goalVelocityLogging.set(currentState.velocity);
        currentVelocityLogging.set(getMotorVelocity());

        turretToTargetDistance.set(ShotCalculator.getInstance().getTurretToTargetDistance());
        turretPose.set(ShotCalculator.getInstance().getTurretPosition());

        if (!stopMoving) followLastProfile();
    }

    public void setPositionProfiling(double degrees, double degreePerSec) {
        goalRotations = getMotorRotFromDegrees(degrees);
        double goalVelocityRotPerSec = getMotorRotFromDegrees(degreePerSec);

        goalState = new TrapezoidProfile.State(goalRotations, goalVelocityRotPerSec);
        currentState = new TrapezoidProfile.State(getMotorRot(), getMotorVelocity());
    }

    public void updateGoalPosition(double degrees, double degreePerSec) {
        goalRotations = getMotorRotFromDegrees(degrees);
        double goalVelocityRotPerSec = getMotorRotFromDegrees(degreePerSec);

        goalState = new TrapezoidProfile.State(goalRotations, goalVelocityRotPerSec);
    }

    public void followLastProfile() {
        TrapezoidProfile.State nextState = profile.calculate(0.02, currentState, goalState);
        setPositionVoltage(nextState.position, getFeedForward(nextState.velocity));
        currentState = nextState;
    }

    public boolean isMechAtGoal() {
        return getMotorRot() >= goalRotations - getMotorRotFromDegrees(TurretConstants.TURRET_LOWER_TOLERANCE)
                && getMotorRot() <= goalRotations + getMotorRotFromDegrees(TurretConstants.TURRET_UPPER_TOLERANCE);
    }

    public void setStopMoving(boolean stopMoving) {
        this.stopMoving = stopMoving;
    }
}