package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.*;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
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

import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;

public class TurretSubsystem extends TemplateSubsystem {
    private static TurretSubsystem turretSubsystem;
    private static Vision vision = Vision.getInstance();
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

    private DoublePublisher goalPositionLogging, goalPositionPhaseDelayed;
    private DoublePublisher currentPositionLogging;
    private DoublePublisher goalVelocityLogging;
    private DoublePublisher currentVelocityLogging;
    private DoublePublisher turretToTargetDistance;
    private DoublePublisher lateralDistance;
    private DoublePublisher velocity;
    private DoublePublisher acceleration;
    private BooleanPublisher isMechAtGoal;
    private StructPublisher<Pose2d> turretPose;
    private StructPublisher<Pose2d> futureTurretPose;

    private PIDController pidController = new PIDController(.5, 0, .15);
    private SimpleMotorFeedforward simpleMotorFeedforward;

    private SingleJointedArmSim turretSim;

    private NetworkTable simulation;
    private DoublePublisher position;
    private StructPublisher<Pose3d> turretSimPose;

    private TalonFXSimState motorSimState;

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
                TurretConstants.SLOT0_CONFIGS);

        // configureSometimesEncoder(TurretConstants.ENCODER_ID,
        //         TurretConstants.CANBUS, TurretConstants.ENCODER_MAGNET_OFFSET,
        //         TurretConstants.SENSOR_TO_MECH_GEAR_RATIO,
        //         TurretConstants.MOTOR_TO_SENSOR_GEAR_RATIO,
        //         TurretConstants.CCW_POSITIVE, TurretConstants.ABSOLUTE_DISCONTINUITY_POINT);

        // configureRoller(TurretConstants.MIN, TurretConstants.MAX);

        // faster();

        profile = new TrapezoidProfile(new TrapezoidProfile
                .Constraints(TurretConstants.VELOCITY, TurretConstants.ACCELERATION));
        currentState = new TrapezoidProfile.State(0, 0);
        goalState = new TrapezoidProfile.State(0, 0);

        networkTable = NetworkTableInstance.getDefault().getTable("AutoTracking/");
        turretNetworkTable = NetworkTableInstance.getDefault().getTable("Subsystems/Turret/");

        goalPositionLogging = networkTable.getDoubleTopic("Goal Position").publish();
        goalPositionPhaseDelayed = networkTable.getDoubleTopic("Goal Position Phase Delay").publish();
        currentPositionLogging = networkTable.getDoubleTopic("Current Position").publish();
//        goalVelocityLogging = networkTable.getDoubleTopic("Goal Velocity").publish();
//        currentVelocityLogging = networkTable.getDoubleTopic("Current Velocity").publish();
//        turretToTargetDistance = networkTable.getDoubleTopic("Distance").publish();
//        lateralDistance = networkTable.getDoubleTopic("lateralDistance").publish();
//        isMechAtGoal = networkTable.getBooleanTopic("Turret Is Mech At Goal").publish();
//        turretPose = networkTable.getStructTopic("Turret Pose", Pose2d.struct).publish();
//        futureTurretPose = networkTable.getStructTopic("Future Turret Pose", Pose2d.struct).publish();
//        velocity = networkTable.getDoubleTopic("Velocity").publish();
//        acceleration = networkTable.getDoubleTopic("Acceleration").publish();

        simpleMotorFeedforward = new SimpleMotorFeedforward(TurretConstants.SLOT0_CONFIGS.kS,
                TurretConstants.SLOT0_CONFIGS.kV, TurretConstants.SLOT0_CONFIGS.kA);

        simulation = NetworkTableInstance.getDefault().getTable("Simulation/");
        position = simulation.getDoubleTopic("Turret Position").publish();
        turretSimPose = NetworkTableInstance.getDefault().getStructTopic("Mechanism/Turret", Pose3d.struct).publish();
        turretSim = new SingleJointedArmSim(
            DCMotor.getKrakenX60Foc(1), 
            getGearRatio(),      
            .01,             // moment of inertia (kg * m^2) - use JVN or CAD
            .1,   // "arm length" - distance from center to edge
            Math.toRadians(TurretConstants.MIN),
            Math.toRadians(TurretConstants.MAX),
            false,
            0
        );

        // motorSimState = getMotor().getSimState();
        // motorSimState.Orientation = ChassisReference.CounterClockwise_Positive;
        // motorSimState.setMotorType(MotorType.KrakenX60);
        // getMotor().setPosition(0);
        // motorSimState.setRawRotorPosition(0);
        // motorSimState.setRotorVelocity(0);
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

        
//        goalVelocityLogging.set(goalVelocityRotPerSec);
//        currentVelocityLogging.set(getMotorVelocity());
//
//        turretToTargetDistance.set(shotCalculator.getTurretToTargetDistance());
//        lateralDistance.set(getLateralDistance());
//        futureTurretPose.set(shotCalculator.getFutureTurretPositionPhaseDelayed());

//        velocity.set(RobotContainer.velocity);
//        acceleration.set(RobotContainer.acceleration);

//        vision.addSample(getDegrees());

//        isMechAtGoal.set(isMechAtGoalAuto());

        if (!stopMoving) followLastProfile();
    }

    public void setPositionProfiling(double degrees, double degreePerSec) {
        goalRotations = getMotorRotFromDegrees(degrees - 90);
        goalVelocityRotPerSec = getMotorRotFromDegrees(degreePerSec);

        goalState = new TrapezoidProfile.State(goalRotations, goalVelocityRotPerSec);
        currentState = new TrapezoidProfile.State(getMotorRot(), getMotorVelocity());
    }

    public void updateGoalPosition(double degrees, double degreePerSec) {
        goalRotations = getMotorRotFromDegrees(degrees - 90);
        goalVelocityRotPerSec = getMotorRotFromDegrees(degreePerSec);

        goalState = new TrapezoidProfile.State(goalRotations, goalVelocityRotPerSec);
    }

    public void followLastProfile() {
        // currentState = profile.calculate(0.02, currentState, goalState);

        // if ((Math.abs(shotCalculator.getTurretAngle() - goalState.position)) >= 10) {
        //     setPositionVoltage(goalState.position, getFF(goalState.velocity));
        // } else {
        //     setPositionVoltage(goalState.position, getFF(goalState.velocity));
        // }
        double volts = pidController.calculate(Math.toDegrees(turretSim.getAngleRads()), getDegreesFromMotorRot(goalState.position));
        volts = MathUtil.clamp(volts, -12, 12);
        // System.out.println("Applied Volts: " + volts);
        turretSim.setInputVoltage(volts);
    }

    public boolean isMechAtGoalAuto() {
        if (predictWrapAround()) return false; //prevents shooting before a wrap

        if (RobotContainer.getShotMode() != ShotMode.SHOOTING) {
            return getDegrees() >= shotCalculator.getTurretAngle() - TurretConstants.LOWER_TOLERANCE
                    && getDegrees() <= shotCalculator.getTurretAngle() + TurretConstants.UPPER_TOLERANCE;
        }
        return getLateralDistance() < .3;
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

    @Override
    public void simulationPeriodic() {
        // runVolts(5);
        // System.out.println("Sim Voltage: " + getMotor().getSimState().getMotorVoltage());
        // System.out.println("Goal rotations: " + goalState.position);
        // System.out.println("Current Position: " + getMotorRot());
        // System.out.println("Current Voltage Calculated: " + motorSimState.getMotorVoltageMeasure().baseUnitMagnitude());
        // turretSimPose.set(update());
        // turretSim.update(.02);
        // position.set(Math.toDegrees(turretSim.getAngleRads()));

        // System.out.println("ClosedLoopError: " + getMotor().getClosedLoopError().getValueAsDouble());
        // System.out.println("ClosedLoopReference: " + getMotor().getClosedLoopReference().getValueAsDouble());
        // System.out.println("RotorPosition: " + getMotor().getRotorPosition().getValueAsDouble());

        // motorSimState.setSupplyVoltage(12);
        // turretSim.setInputVoltage(motorSimState.getMotorVoltageMeasure().baseUnitMagnitude());
        turretSim.update(.02);

        // motorSimState.setRawRotorPosition(getMotorRotFromDegrees(Math.toDegrees(turretSim.getAngleRads())));
        // motorSimState.setRotorVelocity(getMotorRotFromDegrees(Math.toDegrees(turretSim.getVelocityRadPerSec())));

        turretSimPose.set(update());

        goalPositionLogging.set(shotCalculator.getTurretAngle());
        goalPositionPhaseDelayed.set(shotCalculator.getTurretAnglePhaseDelayed());
        currentPositionLogging.set(Math.toDegrees(turretSim.getAngleRads()) + 90);

    }

    public Pose3d update() {
        return new Pose3d(new Translation3d(.22, .1335, 0), new Rotation3d(0, 0, turretSim.getAngleRads()));
    }

    public void runVolts(double volts) {
        turretSim.setInputVoltage(volts);
        
    }

    public void resetState() {
        runVolts(0);
        turretSim.setState(0, 0);
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