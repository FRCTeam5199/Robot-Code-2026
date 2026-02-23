package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.*;
import frc.robot.constants.TurretConstants;
import frc.robot.subsystems.templates.TemplateSubsystem;
import frc.robot.utility.ShotCalculator;
import frc.robot.utility.Type;

public class TurretSubsystem extends TemplateSubsystem {
    private static TurretSubsystem turretSubsystem;
    private static ShotCalculator shotCalculator = ShotCalculator.getInstance();

    private TrapezoidProfile profile;
    private TrapezoidProfile.State currentState;
    private TrapezoidProfile.State goalState;

    private double goalRotations;
    private double goalDegreesWithoutPhaseDelay;
    private double goalVelocityRotPerSec;
    private boolean continuousMotion = false;

    private boolean stopMoving = false;

    private NetworkTable networkTable;
    private NetworkTable turretNetworkTable;

    private DoublePublisher goalPositionLogging;
    private DoublePublisher goalPositionWithoutPhaseDelayLogging;
    private DoublePublisher currentPositionLogging;
    private DoublePublisher goalVelocityLogging;
    private DoublePublisher currentVelocityLogging;
    private DoublePublisher turretToTargetDistance;
    private BooleanPublisher isMechAtGoal;
    private StructPublisher<Pose2d> turretPose;
    private StructPublisher<Pose2d> futureTurretPose;

    private TurretSubsystem() {
        super(Type.ROLLER, TurretConstants.MOTOR_ID,
                TurretConstants.VELOCITY, TurretConstants.ACCELERATION,
                TurretConstants.JERK,
                TurretConstants.LOWER_TOLERANCE,
                TurretConstants.UPPER_TOLERANCE,
                TurretConstants.GEAR_RATIO, "Turret");

        configureMotor(TurretConstants.INVERTED, TurretConstants.BRAKE,
                TurretConstants.SUPPLY_CURRENT_LIMIT,
                TurretConstants.STATOR_CURRENT_LIMIT,
                TurretConstants.SLOT0_CONFIGS);

        configureSometimesEncoder(TurretConstants.ENCODER_ID,
                "rio", TurretConstants.ENCODER_MAGNET_OFFSET,
                TurretConstants.SENSOR_TO_MECH_GEAR_RATIO,
                TurretConstants.MOTOR_TO_SENSOR_GEAR_RATIO,
                TurretConstants.CCW_POSITIVE, TurretConstants.ABSOLUTE_DISCONTINUITY_POINT);

        configureRoller(TurretConstants.MIN, TurretConstants.MAX);

        configureCustomFF();

        profile = new TrapezoidProfile(new TrapezoidProfile
                .Constraints(TurretConstants.VELOCITY, TurretConstants.ACCELERATION));
        currentState = new TrapezoidProfile.State(0, 0);
        goalState = new TrapezoidProfile.State(0, 0);

        networkTable = NetworkTableInstance.getDefault().getTable("AutoTracking/");
        turretNetworkTable = NetworkTableInstance.getDefault().getTable("Subsystems/Turret/");

        goalPositionLogging = networkTable.getDoubleTopic("Goal Position").publish();
        goalPositionWithoutPhaseDelayLogging = networkTable.getDoubleTopic("Goal Position Without Phase Delay").publish();
        currentPositionLogging = networkTable.getDoubleTopic("Current Position").publish();
        goalVelocityLogging = networkTable.getDoubleTopic("Goal Velocity").publish();
        currentVelocityLogging = networkTable.getDoubleTopic("Current Velocity").publish();
        turretToTargetDistance = networkTable.getDoubleTopic("Distance").publish();
        isMechAtGoal = turretNetworkTable.getBooleanTopic("Is Mech At Goal").publish();
        turretPose = networkTable.getStructTopic("Turret Pose", Pose2d.struct).publish();
        futureTurretPose = networkTable.getStructTopic("Future Turret Pose", Pose2d.struct).publish();
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

        goalPositionLogging.set(goalRotations);
        currentPositionLogging.set(getMotorRot());

        goalVelocityLogging.set(goalVelocityRotPerSec);
        currentVelocityLogging.set(getMotorVelocity());

        turretToTargetDistance.set(shotCalculator.getTurretToTargetDistance());

        turretPose.set(shotCalculator.getTurretPosition());
        futureTurretPose.set(shotCalculator.getFutureTurretPosition());

        isMechAtGoal.set(isMechAtGoal());

        if (!stopMoving) followLastProfile();

//        System.out.println("Turret Degrees: " + getDegrees());

    }

    public void setPositionProfiling(double degrees, double degreePerSec) {
        goalRotations = getMotorRotFromDegrees(degrees);
        goalVelocityRotPerSec = getMotorRotFromDegrees(degreePerSec);

        goalState = new TrapezoidProfile.State(goalRotations, goalVelocityRotPerSec);
        currentState = new TrapezoidProfile.State(getMotorRot(), getMotorVelocity());
    }

    public void updateGoalPosition(double degrees, double degreePerSec) {
        goalRotations = getMotorRotFromDegrees(degrees);
        goalVelocityRotPerSec = getMotorRotFromDegrees(degreePerSec);

        goalState = new TrapezoidProfile.State(goalRotations, goalVelocityRotPerSec);
    }

    public void followLastProfile() {
        currentState = profile.calculate(0.02, currentState, goalState);
        setPositionVoltage(goalState.position, getFeedForward(goalState.velocity));
    }

    public boolean isMechAtGoal() {
        return getMotorRot() >= goalRotations - getMotorRotFromDegrees(TurretConstants.LOWER_TOLERANCE)
                && getMotorRot() <= goalRotations + getMotorRotFromDegrees(TurretConstants.UPPER_TOLERANCE);
    }

    public void setStopMoving(boolean stopMoving) {
        this.stopMoving = stopMoving;
    }

    public void setContinuousMotion(boolean continuousMotion) {
        this.continuousMotion = continuousMotion;
    }
}