package frc.robot.subsystems;

import static org.wpilib.units.Units.Rotations;
import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.Second;
import static org.wpilib.units.Units.Seconds;
import static org.wpilib.units.Units.Volts;

import org.wpilib.command2.Command;
import org.wpilib.command2.sysid.SysIdRoutine;
import org.wpilib.math.controller.SimpleMotorFeedforward;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.trajectory.TrapezoidProfile;

import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
import frc.robot.constants.TurretConstants;
import frc.robot.subsystems.templates.TemplateSubsystem;
import frc.robot.utility.AllianceFlipper;
import frc.robot.utility.ShotCalculator;
import frc.robot.utility.ShotMode;
import frc.robot.utility.SubsystemType;


public class TurretSubsystem extends TemplateSubsystem {
    private static TurretSubsystem turretSubsystem;
    private static ShotCalculator shotCalculator = ShotCalculator.getInstance();
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

    public boolean fullStop = false;
    
    private TrapezoidProfile profile;
    private TrapezoidProfile.State currentState;
    private TrapezoidProfile.State goalState;
    private double goalRotations;
    private double goalDegrees;
    private double goalVelocityRotPerSec;
    private boolean continuousMotion = false;
    private boolean stopMoving = true;
    private SimpleMotorFeedforward simpleMotorFeedforward;

    private TurretSubsystem() {
        super(SubsystemType.ROLLER, TurretConstants.MOTOR_ID, TurretConstants.CANBUS,
                TurretConstants.VELOCITY, TurretConstants.ACCELERATION,
                TurretConstants.JERK,
                TurretConstants.LOWER_TOLERANCE,
                TurretConstants.UPPER_TOLERANCE,
                TurretConstants.GEAR_RATIO, "Turret", true);

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

        if (!stopMoving && !fullStop) followLastProfile();
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
        setPositionMotionMagicFF(goalRotations, getFF(goalVelocityRotPerSec));
    }

    public boolean isMechAtGoalAuto() {
        // if (predictWrapAround()) return false;

        if (RobotContainer.getShotMode() != ShotMode.SHOOTING) {
            return getDegrees() >= shotCalculator.getTurretAngle() - TurretConstants.LOWER_TOLERANCE
                    && getDegrees() <= shotCalculator.getTurretAngle() + TurretConstants.UPPER_TOLERANCE;
        }
        return getLateralDistance() < Constants.HUB_RADIUS;
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

    public Command sysIdQuasistaticForward() {
        return sysIdRoutine.quasistatic(SysIdRoutine.Direction.FORWARD);
    }

    public Command sysIdQuasistaticReverse() {
        return sysIdRoutine.quasistatic(SysIdRoutine.Direction.REVERSE);
    }

    public Command sysIdDynamicForward() {
        return sysIdRoutine.dynamic(SysIdRoutine.Direction.FORWARD);
    }

    public Command sysIdDynamicReverse() {
        return sysIdRoutine.dynamic(SysIdRoutine.Direction.REVERSE);
    }
}