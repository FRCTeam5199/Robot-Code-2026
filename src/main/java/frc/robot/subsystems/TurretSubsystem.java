package frc.robot.subsystems;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.*;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
import frc.robot.constants.TurretConstants;
import frc.robot.subsystems.templates.TemplateSubsystem;
import frc.robot.utility.AllianceFlipper;
import frc.robot.utility.ShotCalculator;
import frc.robot.utility.ShotMode;
import frc.robot.utility.Type;

import static edu.wpi.first.units.Units.*;

public class TurretSubsystem extends TemplateSubsystem {
    private static TurretSubsystem turretSubsystem;
    private static Vision vision = Vision.getInstance();
    private static ShotCalculator shotCalculator = ShotCalculator.getInstance();

    private TrapezoidProfile profile;
    private TrapezoidProfile.State currentState;
    private TrapezoidProfile.State goalState;

    private double goalRotations;
    private double goalDegrees;
    private double goalDegreesWithoutPhaseDelay;
    private double goalVelocityRotPerSec;
    private boolean continuousMotion = false;

    private boolean stopMoving = true;

    private NetworkTable networkTable;
    private NetworkTable turretNetworkTable;

    private DoublePublisher goalPositionLogging, goalPositionPhaseDelayed;
    private DoublePublisher currentPositionLogging;
    private DoublePublisher goalVelocityLogging;
    private DoublePublisher currentVelocityLogging;
    private DoublePublisher turretToTargetDistance;
    private DoublePublisher lateralDistance;
    private DoublePublisher velocity;
    private DoublePublisher acceleration;
    private BooleanPublisher isMechAtGoal;
    private BooleanPublisher isShooterAtGoal;
    private BooleanPublisher isIndexerAtGoal;
    private BooleanPublisher isHoodAtGoal;
    private BooleanPublisher areMechanismsAtGoalAuto;
    private DoublePublisher shooterVelocity;
    private DoublePublisher indexerVelocity;
    private DoublePublisher hopperVelocity;
    private StructPublisher<Pose2d> turretPose;
    private StructPublisher<Pose2d> futureTurretPose;

    private SimpleMotorFeedforward simpleMotorFeedforward;
    public boolean fullStop = false;

    private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                    Volts.of(0.2).per(Second), // ramp rate - slow for a turret
                    Volts.of(6),                       // max voltage - keep low for turret safety
                    Seconds.of(8),                     // test timeout
                    null
            ),
            new SysIdRoutine.Mechanism(
                    (voltage) -> setVoltage(voltage.in(Volts)),
                    log -> {
                        log.motor("turret")
                                .voltage(Volts.of(getMotor().getMotorVoltage().getValueAsDouble()))
                                .angularPosition(Rotations.of(getMotor().getRotorPosition().getValueAsDouble()))
                                .angularVelocity(RotationsPerSecond.of(getMotor().getRotorVelocity().getValueAsDouble()));
                    },
                    this
            )
    );

    private TurretSubsystem() {
        super(Type.ROLLER, TurretConstants.MOTOR_ID,
                TurretConstants.VELOCITY, TurretConstants.ACCELERATION,
                TurretConstants.JERK,
                TurretConstants.LOWER_TOLERANCE,
                TurretConstants.UPPER_TOLERANCE,
                TurretConstants.GEAR_RATIO, "Turret", true, TurretConstants.CANBUS);

        configureMotor(TurretConstants.INVERTED, TurretConstants.BRAKE,
                TurretConstants.SUPPLY_CURRENT_LIMIT,
                TurretConstants.STATOR_CURRENT_LIMIT,
                TurretConstants.SLOT0_CONFIGS, true);

        configureSometimesEncoder(TurretConstants.ENCODER_ID,
                TurretConstants.CANBUS, TurretConstants.ENCODER_MAGNET_OFFSET,
                TurretConstants.SENSOR_TO_MECH_GEAR_RATIO,
                TurretConstants.MOTOR_TO_SENSOR_GEAR_RATIO,
                TurretConstants.CCW_POSITIVE, TurretConstants.ABSOLUTE_DISCONTINUITY_POINT);

        configureRoller(TurretConstants.MIN, TurretConstants.MAX);

        profile = new TrapezoidProfile(new TrapezoidProfile
                .Constraints(TurretConstants.VELOCITY, TurretConstants.ACCELERATION));
        currentState = new TrapezoidProfile.State(0, 0);
        goalState = new TrapezoidProfile.State(0, 0);

        networkTable = NetworkTableInstance.getDefault().getTable("AutoTracking/");
        turretNetworkTable = NetworkTableInstance.getDefault().getTable("Subsystems/Turret/");

        goalPositionLogging = networkTable.getDoubleTopic("Goal Position").publish();
        goalPositionPhaseDelayed = networkTable.getDoubleTopic("Goal Position Phase Delay").publish();
        currentPositionLogging = networkTable.getDoubleTopic("Current Position").publish();

//        shooterVelocity = networkTable.getDoubleTopic("Shooter Velocity").publish();
//        indexerVelocity = networkTable.getDoubleTopic("Indexer Velocity").publish();
//        hopperVelocity = networkTable.getDoubleTopic("Hopper Velocity").publish();

//        goalVelocityLogging = networkTable.getDoubleTopic("Goal Velocity").publish();
//        currentVelocityLogging = networkTable.getDoubleTopic("Current Velocity").publish();
        turretToTargetDistance = networkTable.getDoubleTopic("Distance").publish();
//        lateralDistance = networkTable.getDoubleTopic("lateralDistance").publish();
        isMechAtGoal = networkTable.getBooleanTopic("Turret Is Mech At Goal").publish();
        isHoodAtGoal = networkTable.getBooleanTopic("Hood Is Mech At Goal").publish();
        isShooterAtGoal = networkTable.getBooleanTopic("Shooter Is Mech At Goal").publish();
//        isIndexerAtGoal = networkTable.getBooleanTopic("Indexer Is Mech At Goal").publish();
//        areMechanismsAtGoalAuto = networkTable.getBooleanTopic("Mechanisms Are At Goal").publish();
//        turretPose = networkTable.getStructTopic("Turret Pose", Pose2d.struct).publish();
       futureTurretPose = networkTable.getStructTopic("Future Turret Pose", Pose2d.struct).publish();
//        velocity = networkTable.getDoubleTopic("Velocity").publish();
//        acceleration = networkTable.getDoubleTopic("Acceleration").publish();

        simpleMotorFeedforward = new SimpleMotorFeedforward(TurretConstants.SLOT0_CONFIGS.kS,
                TurretConstants.SLOT0_CONFIGS.kV, TurretConstants.SLOT0_CONFIGS.kA);
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

        // goalPositionLogging.set(shotCalculator.getHoodAngle());
        // goalPositionPhaseDelayed.set(shotCalculator.getHoodAnglePhaseDelayed());
        // currentPositionLogging.set(HoodSubsystem.getInstance().getDegrees());

//        shooterVelocity.set(ShooterSubsystem.getInstance().getMotorVelocity());
//        indexerVelocity.set(KickerSubsystem.getInstance().getMotorVelocity());
//        hopperVelocity.set(HopperSubsystem.getInstance().getMotorVelocity());
//
        // isMechAtGoal.set(isMechAtGoalAuto());
        // isHoodAtGoal.set(HoodSubsystem.getInstance().isMechAtGoalAuto());
        // isShooterAtGoal.set(ShooterSubsystem.getInstance().isMechAtGoalAuto());
//        isIndexerAtGoal.set(KickerSubsystem.getInstance().isMechAtGoalAuto());
//        areMechanismsAtGoalAuto.set(RobotContainer.areMechanismsAtGoalsAuto());
//
//        goalVelocityLogging.set(goalVelocityRotPerSec);
//        currentVelocityLogging.set(getMotorVelocity());
//
        // turretToTargetDistance.set(shotCalculator.getTurretToTargetDistance());
//        lateralDistance.set(getLateralDistance());
//        futureTurretPose.set(shotCalculator.getFutureTurretPositionPhaseDelayed());

//        velocity.set(RobotContainer.velocity);
//        acceleration.set(RobotContainer.acceleration);

//        vision.addSample(getDegrees());

//        isMechAtGoal.set(isMechAtGoalAuto());

        if (!stopMoving && !fullStop) followLastProfile();
        // System.out.println(isMechAtGoalAuto());
//        System.out.println(getDegrees());
//        System.out.println("goal: " + getGoal());
    }

    public void setPositionProfiling(double degrees, double degreePerSec) {
        goalRotations = getMotorRotFromDegrees(degrees);
        goalDegrees = degrees;
        goalVelocityRotPerSec = getMotorRotFromDegrees(degreePerSec);

        goalState = new TrapezoidProfile.State(goalRotations, goalVelocityRotPerSec);
        currentState = new TrapezoidProfile.State(getMotorRot(), getMotorVelocity());
    }

    public void updateGoalPosition(double degrees, double degreePerSec) {
        goalRotations = getMotorRotFromDegrees(degrees);
        goalDegrees = degrees;
        goalVelocityRotPerSec = getMotorRotFromDegrees(degreePerSec);

        goalState = new TrapezoidProfile.State(goalRotations, goalVelocityRotPerSec);
    }

    public void followLastProfile() {
//        currentState = profile.calculate(0.02, currentState, goalState);
//
//        if ((Math.abs(shotCalculator.getTurretAngle() - goalState.position)) >= 10) {
//            setPositionVoltage(currentState.position, getFF(currentState.velocity));
//        } else {
//            setPositionVoltage(currentState.position, getFF(currentState.velocity));
//        }
        setPositionMotionMagicFF(goalRotations, getFF(goalVelocityRotPerSec));
    }

    public boolean isMechAtGoalAuto() {
//        if (predictWrapAround()) return false; //prevents shooting before a wrap

        if (RobotContainer.getShotMode() != ShotMode.SHOOTING) {
            return getDegrees() >= shotCalculator.getTurretAngle() - TurretConstants.LOWER_TOLERANCE
                    && getDegrees() <= shotCalculator.getTurretAngle() + TurretConstants.UPPER_TOLERANCE;
        }
        return getLateralDistance() < Constants.HUB_RADIUS - .1;
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

    public double getLateralDistance() {
        double degrees = getDegrees() + RobotContainer.getPose().getRotation().getDegrees();
        double slope = Math.tan(Math.toRadians(degrees));
        Pose2d futureTurretPose = shotCalculator.getFutureTurretPosition();

        Translation2d hubCenter = AllianceFlipper.getCorrectAlliance(Constants.BLUE_HUB_CENTER,
                Constants.RED_HUB_CENTER);

        double deltaX = hubCenter.getX() - futureTurretPose.getX();
        double deltaY = slope * deltaX;
        double projectedY = futureTurretPose.getY() + deltaY;

        return Math.abs(hubCenter.getY() - projectedY);
    }

    public double getFF(double velocity) {
        return simpleMotorFeedforward.calculate(velocity);
    }


    public static boolean predictWrapAround() {
        double predictedTurretPosition = turretSubsystem.getDegrees()
                + turretSubsystem.getDegreesFromMotorRot(turretSubsystem.getMotorVelocity())
                * TurretConstants.INDEXING_TIME;
        return predictedTurretPosition >= TurretConstants.MAX
                || predictedTurretPosition <= TurretConstants.MIN;
    }


    public Command sysIdQuasistaticForward() {
        return sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward);
    }

    public Command sysIdQuasistaticReverse() {
        return sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse);
    }

    public Command sysIdDynamicForward() {
        return sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward);
    }

    public Command sysIdDynamicReverse() {
        return sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse);
    }
}