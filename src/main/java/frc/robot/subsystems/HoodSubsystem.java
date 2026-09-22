package frc.robot.subsystems;

import org.wpilib.math.controller.SimpleMotorFeedforward;
import org.wpilib.math.trajectory.TrapezoidProfile;
import org.wpilib.networktables.BooleanPublisher;
import org.wpilib.networktables.DoublePublisher;
import org.wpilib.networktables.NetworkTable;

import frc.robot.RobotContainer;
import frc.robot.constants.HoodConstants;
import frc.robot.subsystems.templates.TemplateSubsystem;
import frc.robot.utility.ShotCalculator;
import frc.robot.utility.ShotMode;
import frc.robot.utility.SubsystemType;

public class HoodSubsystem extends TemplateSubsystem {
    private static HoodSubsystem hoodSubsystem;

    private TrapezoidProfile profile;
    private TrapezoidProfile.State currentState;
    private TrapezoidProfile.State goalState;

    private double goalRotations;
    private double goalVelocityRotPerSec;
    private boolean continuousMotion = false;

    private boolean stopMoving = true;

    private NetworkTable hoodTable;
    private DoublePublisher goalPosition;
    private DoublePublisher goalPositionPhaseDelay;
    private DoublePublisher currentPosition;
    private BooleanPublisher isMechAtGoal;
    private SimpleMotorFeedforward simpleMotorFeedforward;

    private HoodSubsystem() {
        super(SubsystemType.PIVOT, HoodConstants.MOTOR_ID, HoodConstants.CANBUS, HoodConstants.VELOCITY,
                HoodConstants.ACCELERATION, HoodConstants.JERK,
                HoodConstants.LOWER_TOLERANCE,
                HoodConstants.UPPER_TOLERANCE,
                HoodConstants.GEAR_RATIO, "Hood", true);

        configureMotor(HoodConstants.INVERTED, HoodConstants.BRAKE,
                HoodConstants.SUPPLY_CURRENT_LIMIT,
                HoodConstants.STATOR_CURRENT_LIMIT,
                HoodConstants.SLOT0_CONFIGS, true);

        configurePivot(HoodConstants.MIN,
                HoodConstants.MAX);

        profile = new TrapezoidProfile(new TrapezoidProfile
                .Constraints(HoodConstants.VELOCITY, HoodConstants.ACCELERATION));
        currentState = new TrapezoidProfile.State(0, 0);
        goalState = new TrapezoidProfile.State(0, 0);

        simpleMotorFeedforward = new SimpleMotorFeedforward(HoodConstants.SLOT0_CONFIGS.kS,
                HoodConstants.SLOT0_CONFIGS.kV, HoodConstants.SLOT0_CONFIGS.kA);
    }

    public static HoodSubsystem getInstance() {
        if (hoodSubsystem == null) {
            hoodSubsystem = new HoodSubsystem();
        }
        return hoodSubsystem;
    }

    public void periodic() {
        super.periodic();
        if (!stopMoving) followLastProfile();
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
        setPositionMotionMagicFF(goalRotations, getFF(goalVelocityRotPerSec));
    }

    public boolean isMechAtGoalAuto() {
        if (RobotContainer.getShotMode() != ShotMode.SHOOTING) return true;
        return getDegrees() >= ShotCalculator.getInstance().getHoodAngle() - HoodConstants.LOWER_TOLERANCE
                && getDegrees() <= ShotCalculator.getInstance().getHoodAngle() + HoodConstants.UPPER_TOLERANCE;
    }

    public boolean isMechAtGoal() {
        return getMotorRot() >= goalRotations - getMotorRotFromDegrees(HoodConstants.LOWER_TOLERANCE)
                && getMotorRot() <= goalRotations + getMotorRotFromDegrees(HoodConstants.UPPER_TOLERANCE);
    }

    public void setStopMoving(boolean stopMoving) {
        this.stopMoving = stopMoving;
    }

    public void setContinuousMotion(boolean continuousMotion) {
        this.continuousMotion = continuousMotion;
    }

    public double getFF(double velocity) {
        return simpleMotorFeedforward.calculate(velocity);
    }
}

