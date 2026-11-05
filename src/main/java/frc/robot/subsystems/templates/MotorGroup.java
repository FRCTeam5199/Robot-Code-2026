package frc.robot.subsystems.templates;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.utility.Type;

/**
 * A leader motor, any number of followers, and everything that belongs to them: goal, status
 * signals, config, gear ratio and tolerances.
 *
 * <p>Put motors in the same group when they should always get the same output, such as motors on
 * the same shaft. Motors that need different outputs go in separate groups on the same
 * {@link TemplateSubsystem}.
 */
public class MotorGroup {
    private final TalonFX motor;
    private final List<TalonFX> followers = new ArrayList<>();
    private final CANBus canbus;
    private final StatusSignal<Angle> positionStatusSignal;
    private final StatusSignal<AngularVelocity> velocityStatusSignal;
    // Logging
    private NetworkTable networkTable;
    private DoublePublisher poseData;
    private DoublePublisher velocityData;
    private DoublePublisher voltageData;
    private DoublePublisher supplyCurrentData;
    private DoublePublisher statorCurrentData;
    private DoublePublisher tempData;
    private TalonFXConfiguration motorConfig;
    private CANcoder encoder;
    private CANcoderConfiguration encoderConfig;
    private CANcoder sometimesEncoder;
    private CANcoderConfiguration sometimesEncoderConfig;
    private double goal;
    private boolean followLastMechProfile = false;
    private MotionMagicVoltage motionMagicVoltage;
    private MotionMagicVelocityVoltage motionMagicVelocityVoltage;
    private PositionVoltage positionVoltage;
    private VelocityVoltage velocityVoltage;
    private Slot0Configs slot0Configs;
    private double velocity;
    private double acceleration;
    private double jerk;
    private double lowerTolerance;
    private double upperTolerance;
    private double sensorToMechRatio;
    private double offset;
    private boolean changedOffset = false;
    private double gearRatio = 1d;
    private double motorToSensorRatio = 1d;
    private double drumCircumference;
    private Type type;

    public MotorGroup(Type type, int id, double velocity, double acceleration, double jerk,
                      double lowerTolerance, double upperTolerance,
                      double[][] gearRatios, boolean enableFoc) {
        this(type, id, velocity, acceleration, jerk, lowerTolerance, upperTolerance,
                gearRatios, enableFoc, new CANBus());
    }

    public MotorGroup(Type type, int id, double velocity, double acceleration, double jerk,
                      double lowerTolerance, double upperTolerance,
                      double[][] gearRatios, boolean enableFoc, CANBus canbus) {
        this.type = type;
        this.canbus = canbus;

        motor = new TalonFX(id, canbus);
        motorConfig = new TalonFXConfiguration();

        positionStatusSignal = motor.getRotorPosition();
        velocityStatusSignal = motor.getRotorVelocity();

        this.velocity = velocity;
        this.acceleration = acceleration;
        this.jerk = jerk;

        motionMagicVoltage = new MotionMagicVoltage(0).withSlot(0).withEnableFOC(enableFoc);
        motionMagicVelocityVoltage = new MotionMagicVelocityVoltage(0).withSlot(0)
                .withEnableFOC(enableFoc);
        positionVoltage = new PositionVoltage(0).withSlot(0)
                .withEnableFOC(enableFoc);
        velocityVoltage = new VelocityVoltage(0).withSlot(0)
                .withEnableFOC(enableFoc);

        this.lowerTolerance = lowerTolerance;
        this.upperTolerance = upperTolerance;

        for (double[] ratio : gearRatios) {
            this.gearRatio *= (ratio[1] / ratio[0]);
        }

        /* Subsystem Logging on AdvantageKit (each group needs its own table name if re-enabled) */
//        networkTable = NetworkTableInstance.getDefault().getTable("Subsystems/" + SubsystemName);
//
//        poseData = networkTable.getDoubleTopic("Position").publish();
//        velocityData = networkTable.getDoubleTopic("Velocity").publish();
//        voltageData = networkTable.getDoubleTopic("Voltage").publish();
//        supplyCurrentData = networkTable.getDoubleTopic("SupplyCurrent").publish();
//        statorCurrentData = networkTable.getDoubleTopic("StatorCurrent").publish();
//        tempData = networkTable.getDoubleTopic("Temp").publish();
    }

    //Configurations
    public void configureMotor(boolean isInverted, boolean isBrakeMode,
                               double supplyCurrentLimit, double statorCurrentLimit,
                               Slot0Configs slot0Configs, boolean updateFaster) {
        motorConfig.MotorOutput.Inverted =
                isInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        motorConfig.MotorOutput.NeutralMode = isBrakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        motorConfig.CurrentLimits.SupplyCurrentLimit = supplyCurrentLimit;
        motorConfig.CurrentLimits.StatorCurrentLimit = statorCurrentLimit;
        motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        motorConfig.Slot0 = slot0Configs;
        this.slot0Configs = slot0Configs;

        motorConfig.MotionMagic.MotionMagicCruiseVelocity = velocity;
        motorConfig.MotionMagic.MotionMagicAcceleration = acceleration;
        motorConfig.MotionMagic.MotionMagicJerk = jerk;

        if (updateFaster) {
            motorConfig.MotorOutput.ControlTimesyncFreqHz = 200;
            positionStatusSignal.setUpdateFrequency(200);
            velocityStatusSignal.setUpdateFrequency(200);
        } else {
//            positionStatusSignal.setUpdateFrequency(50);
            velocityStatusSignal.setUpdateFrequency(50);
        }

        motor.optimizeBusUtilization();

        motor.getConfigurator().apply(motorConfig);
        motor.setPosition(0);
    }

    /**
     * Adds a motor that copies the leader's output. Call once per follower, after configureMotor().
     *
     * @param followerId   CAN ID of the follower, on the same bus as the leader
     * @param opposeLeader true if this motor spins opposite the leader
     */
    public void addFollower(int followerId, boolean opposeLeader) {
        TalonFX follower = new TalonFX(followerId, canbus);

        // Followers keep their own current limits and neutral mode, so give them the leader's.
        TalonFXConfiguration followerConfig = new TalonFXConfiguration();
        followerConfig.CurrentLimits = motorConfig.CurrentLimits;
        followerConfig.MotorOutput.NeutralMode = motorConfig.MotorOutput.NeutralMode;
        follower.getConfigurator().apply(followerConfig);

        // A follower copies one of these leader signals, so they must stay enabled and fast.
        BaseStatusSignal.setUpdateFrequencyForAll(100,
                motor.getDutyCycle(), motor.getMotorVoltage(), motor.getTorqueCurrent());

        follower.setControl(new Follower(motor.getDeviceID(),
                opposeLeader ? MotorAlignmentValue.Opposed : MotorAlignmentValue.Aligned));
        follower.optimizeBusUtilization();
        followers.add(follower);
    }

    public void configureLinearMech(double drumCircumference, double motorMinRotation, double motorMaxRotation) {
        this.drumCircumference = drumCircumference;
        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = getMotorRotFromMechM(motorMaxRotation);
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = getMotorRotFromMechM(motorMinRotation);

        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        // motor.getConfigurator().apply(motorConfig);
    }

    public void configurePivot(double motorMinDegrees, double motorMaxDegrees) {
        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = getEncoderRotFromDegrees(motorMaxDegrees);
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = getEncoderRotFromDegrees(motorMinDegrees);

        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        motor.getConfigurator().apply(motorConfig);
    }

    public void configureRoller(double motorMinDegrees, double motorMaxDegrees) {
        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = getMotorRotFromDegrees(motorMaxDegrees);
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = getMotorRotFromDegrees(motorMinDegrees);

        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        motor.getConfigurator().apply(motorConfig);
    }

    public void configureEncoder(int encoderId, String canbus, double magnetOffset,
                                 double sensorToMechRatio, double motorToSensorRatio, boolean isCCWPositive) {
        encoder = new CANcoder(encoderId);
        encoderConfig = new CANcoderConfiguration();

        encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = .5;
        encoderConfig.MagnetSensor.SensorDirection = isCCWPositive ? SensorDirectionValue.CounterClockwise_Positive
                : SensorDirectionValue.Clockwise_Positive;

        encoderConfig.MagnetSensor.MagnetOffset = magnetOffset;

        encoder.getConfigurator().apply(encoderConfig);

        motorConfig.Feedback.FeedbackRemoteSensorID = encoderId;
        motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;

        motorConfig.Feedback.SensorToMechanismRatio = sensorToMechRatio;
        motorConfig.Feedback.RotorToSensorRatio = motorToSensorRatio;

        this.sensorToMechRatio = sensorToMechRatio;

        motor.getConfigurator().apply(motorConfig);
        gearRatio = motorToSensorRatio * sensorToMechRatio;
    }

    public void configureSometimesEncoder(int encoderId, CANBus canBus, double magnetOffset,
                                          double sensorToMechRatio, double motorToSensorRatio,
                                          boolean isCCWPositive, double absoluteDiscontinuityPoint) {
        sometimesEncoder = new CANcoder(encoderId, canBus);
        sometimesEncoderConfig = new CANcoderConfiguration();

        sometimesEncoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = absoluteDiscontinuityPoint;
        sometimesEncoderConfig.MagnetSensor.SensorDirection = isCCWPositive ? SensorDirectionValue.CounterClockwise_Positive
                : SensorDirectionValue.Clockwise_Positive;

        sometimesEncoderConfig.MagnetSensor.MagnetOffset = magnetOffset;
        sometimesEncoder.getConfigurator().apply(sometimesEncoderConfig);

        this.motorToSensorRatio = motorToSensorRatio;

        motor.setPosition(sometimesEncoder.getAbsolutePosition().getValueAsDouble() * motorToSensorRatio);
    }

    public void setBrake(boolean brake) {
        NeutralModeValue mode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        motor.setNeutralMode(mode);
        followers.forEach(follower -> follower.setNeutralMode(mode));
    }

    public void setPercent(double percent) {
        followLastMechProfile = false;
        if (percent > 1) percent /= 100d;
        motor.set(percent);
    }

    public void setVoltage(double output) {
        followLastMechProfile = false;
        motor.setVoltage(output);
    }

    public void setVelocity(double rps) {
        this.goal = rps;
        followLastMechProfile = false;
        if (rps == 0) setPercent(0);
        else motor.setControl(velocityVoltage.withVelocity(rps + offset));
    }

    public void setPosition(double goal) {
        double goalRotations;

        if (type == Type.LINEAR) {
            goalRotations = getMotorRotFromMechM(goal + offset);
        } else {
            goalRotations = encoder == null ? getMotorRotFromDegrees(goal + offset)
                    : getEncoderRotFromDegrees(goal + offset);
        }

        this.goal = goal;

        motor.setControl(motionMagicVoltage.withPosition(goalRotations));

//        motionMagicVoltage.Velocity = this.velocity;
//        motionMagicVoltage.Acceleration = this.acceleration;
//        motionMagicVoltage.Jerk = this.jerk;
    }

    public void setPositionMotionMagicFF(double goalRotations, double ff) {
        this.goal = getDegreesFromMotorRot(goalRotations);
        motor.setControl(motionMagicVoltage.withPosition(goalRotations));
    }

    public void setPosition(double goal, double feedforward) {
        double goalRotations;

        if (type == Type.LINEAR) {
            goalRotations = getMotorRotFromMechM(goal + offset);
        } else {
            goalRotations = encoder == null ? getMotorRotFromDegrees(goal + offset)
                    : getEncoderRotFromDegrees(goal + offset);
        }

        this.goal = goal;
        motor.setControl(motionMagicVoltage.withPosition(goalRotations)
                .withFeedForward(feedforward));

//        motionMagicVoltage.Velocity = this.velocity;
//        motionMagicVoltage.Acceleration = this.acceleration;
//        motionMagicVoltage.Jerk = this.jerk;
    }

    public void setPositionVoltage(double motorRotations, double feedforward) {
        motor.setControl(positionVoltage.withPosition(motorRotations)
                .withFeedForward(feedforward));
    }

    public void setPositionVoltage(double motorRotations) {
        motor.setControl(positionVoltage.withPosition(motorRotations));
    }

    //Used if velocity/acceleration/jerk constraint needs to be changed
    public void setPosition(double goal, double velocity, double acceleration, double jerk) {
        double goalRotations;

        if (type == Type.LINEAR) {
            goalRotations = getMotorRotFromMechM(goal + offset);
        } else {
            goalRotations = encoder == null ? getMotorRotFromDegrees(goal + offset)
                    : getEncoderRotFromDegrees(goal + offset);
        }

        this.goal = goal;


        motor.setControl(motionMagicVoltage.withPosition(goalRotations));
    }

    public void setConstraints(double velocity, double acceleration, double jerk) {
//        motionMagicVoltage.Velocity = velocity;
//        motionMagicVoltage.Acceleration = acceleration;
//        motionMagicVoltage.Jerk = jerk;
    }

    public boolean isMechAtGoal(boolean isVelocity) {
        switch (type) {
            case LINEAR -> {
                return getMechM() >= goal - lowerTolerance && getMechM() <= goal + upperTolerance;
            }
            case PIVOT -> {
                return getDegrees() >= goal - lowerTolerance && getDegrees() <= goal + upperTolerance;
            }
            default -> {
                if (isVelocity) return getMotorVelocity() >= goal - lowerTolerance
                        && getMotorVelocity() <= goal + upperTolerance;
                else return getDegrees() >= goal - lowerTolerance
                        && getDegrees() <= goal + upperTolerance;
            }
        }
    }

    public boolean isMechGreaterThanPosition(double position) {
        switch (type) {
            case LINEAR -> {
                return getMechM() >= position;
            }
            case PIVOT -> {
                return getDegrees() >= position;
            }
            default -> {
                return false;
            }
        }
    }

    public boolean isAboveSpeed(double goalVelocity) {
        if (type != Type.ROLLER) return false;
        return getMechVelocity() > goalVelocity;
    }

    public void setOffset(double offset, boolean changedOffset) {
        this.offset = offset;
        this.changedOffset = changedOffset;
    }

    public double getOffset() {
        return offset;
    }

    public void setOffset(double offset) {
        this.offset = offset;
        changedOffset = true;
    }

    public void changeOffset(double deltaOffset) {
        this.offset += deltaOffset;
        this.changedOffset = true;
    }

    public double getGoal() {
        return goal;
    }

    public void setFollowLastMechProfile(boolean followLastMechProfile) {
        this.followLastMechProfile = followLastMechProfile;
    }


    //Unit Conversions
    public double getDegrees() {
        return encoder == null ? positionStatusSignal.getValueAsDouble() * gearRatio * 360d
                : getEncoderDegrees();
    }

    /**
     * @return Gets the position of the actual motor of the mechanism.
     */

    public double getMotorRot() {
//        BaseStatusSignal.refreshAll(positionStatusSignal, velocityStatusSignal);
//        return BaseStatusSignal.getLatencyCompensatedValueAsDouble(positionStatusSignal, velocityStatusSignal);
        return positionStatusSignal.getValueAsDouble();
    }

    /**
     * @return The rotation of the mechanism itself (Accounts for gear ratios and stuff)
     */
    public double getMechRot() {
        return positionStatusSignal.getValueAsDouble() * gearRatio;
    }

    public double getDegreesFromMechRot(double mechRot) {
        return mechRot * 360d;
    }

    public double getDegreesFromMotorRot(double motorRot) {
        return motorRot * gearRatio * 360d;
    }

    public double getMechRotFromDegrees(double degrees) {
        return degrees / 360d;
    }

    public double getMechRotFromMotorRot(double motorRot) {
        return motorRot * gearRatio;
    }

    public double getMotorRotFromDegrees(double degrees) {
        return degrees / 360d / gearRatio;
    }

    public double getMotorRotFromMechRot(double mechRot) {
        return mechRot / gearRatio;
    }

    /**
     * @return the current height of the elevator as a double
     */
    public double getMechM() {
        if (type != Type.LINEAR) return 0;
        return positionStatusSignal.getValueAsDouble() * drumCircumference * gearRatio;
    }

    /**
     * @return a constantly updating value for the height of the elevator as a double.
     */
    public DoubleSupplier getMechMeter() {
        if (type != Type.LINEAR) return () -> 0;
        return () -> positionStatusSignal.getValueAsDouble() * drumCircumference * gearRatio;
    }

    public double getMechMFromMotorRot(double motorRot) {
        if (type != Type.LINEAR) return 0;
        return motorRot * drumCircumference * gearRatio;
    }

    public double getMotorRotFromMechM(double mechM) {
        if (type != Type.LINEAR) return 0;
        return mechM / drumCircumference / gearRatio;
    }

    //Motor Values
    public double getMotorVelocity() {
        return velocityStatusSignal.getValueAsDouble();
    }

    public double getMotorVoltage() {
        return motor.getMotorVoltage().getValueAsDouble();
    }

    public double getMotorTemp() {
        return motor.getDeviceTemp().getValueAsDouble();
    }

    public TalonFX getMotor() {
        return motor;
    }

    public List<TalonFX> getFollowers() {
        return Collections.unmodifiableList(followers);
    }

    public double getSupplyVoltage() {
        return motor.getSupplyVoltage().getValueAsDouble();
    }

    public double getSupplyCurrent() {
        return motor.getSupplyCurrent().getValueAsDouble();
    }

    public double getStatorCurrent() {
        return motor.getStatorCurrent().getValueAsDouble();
    }

    public double getMechVelocity() {
        return getMechRotFromMotorRot(motor.getVelocity().getValueAsDouble());
    }

    public CANcoder getEncoder() {
        return encoder;
    }

    public double getEncoderRot() {
        return encoder.getAbsolutePosition().getValueAsDouble();
    }

    public double getEncoderRotFromDegrees(double degrees) {
        if (encoder == null) return getMotorRotFromDegrees(degrees);
        return degrees / 360 * sensorToMechRatio;
    }

    public double getDegreesFromEncoderRot(double encoderRot) {
        if (encoder == null) return getDegreesFromMotorRot(encoderRot);
        return encoderRot * 360 * sensorToMechRatio;
    }

    public double getEncoderDegrees() {
        return encoder.getAbsolutePosition().getValueAsDouble() * 360d * sensorToMechRatio;
    }

    public double getSometimesEncoderRot() {
        if (sometimesEncoder == null) return 0;
        return sometimesEncoder.getAbsolutePosition().getValueAsDouble();
    }

    public void zeroMotor() {
        if (sometimesEncoder == null) motor.setPosition(0);
        else motor.setPosition(sometimesEncoder.getAbsolutePosition().getValueAsDouble() * this.motorToSensorRatio);
    }

    /** Called every loop by {@link TemplateSubsystem#periodic()}. */
    void periodic() {
        if (changedOffset) {
            setPosition(goal);
            changedOffset = false;
        }

        BaseStatusSignal.refreshAll(positionStatusSignal, velocityStatusSignal);

//        if (type == Type.LINEAR) poseData.set(getMechM());
//        else poseData.set(getDegrees());
//        velocityData.set(getMotorVelocity());
//        voltageData.set(getMotorVoltage());
//        supplyCurrentData.set(-getSupplyCurrent());
//        statorCurrentData.set(-getStatorCurrent());
//        tempData.set(getMotorTemp());
    }

    public void setControl(ControlRequest control) {
        motor.setControl(control);
    }

    public double getGearRatio() {
        return gearRatio;
    }
}
