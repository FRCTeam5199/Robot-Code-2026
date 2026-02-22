package frc.robot.subsystems.templates;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utility.Type;

public class TemplateSubsystem extends SubsystemBase {
    private NetworkTable networkTable;
    private DoublePublisher poseData;
    private DoublePublisher velocityData;
    private DoublePublisher voltageData;
    private DoublePublisher supplyCurrentData;
    private DoublePublisher statorCurrentData;
    private DoublePublisher tempData;
    private TalonFX motor;
    private TalonFXConfiguration motorConfig;
    private TalonFX followerMotor;
    private Follower follower;
    private TalonFX secondaryMotor;
    private TalonFXConfiguration secondaryMotorConfig;
    private CANcoder encoder;
    private CANcoderConfiguration encoderConfig;
    private CANcoder sometimesEncoder;
    private CANcoderConfiguration sometimesEncoderConfig;
    private double goal;
    private double secondaryGoal;
    private boolean followLastMechProfile = false;
    private boolean isCommandRunning = false;
    private DynamicMotionMagicVoltage dynamicMotionMagicVoltage;
    private MotionMagicVelocityVoltage motionMagicVelocityVoltage;
    private MotionMagicVelocityVoltage secondaryMotionMagicVelocityVoltage;
    private PositionVoltage positionVoltage;
    private VelocityVoltage velocityVoltage;
    private SimpleMotorFeedforward simpleMotorFeedforward;
    private Slot0Configs slot0Configs;
    private double velocity;
    private double acceleration;
    private double jerk;
    private SimpleMotorFeedforward secondarySimpleMotorFF;
    private double lowerTolerance;
    private double upperTolerance;
    private double sensorToMechRatio;
    private double offset;
    private boolean changedOffset = false;
    private double gearRatio = 1d;
    private double motorToSensorRatio = 1d;
    private double drumCircumference;
    private Type type;
    private String name;

    public TemplateSubsystem(Type type, int id, double velocity, double acceleration, double jerk,
                             double lowerTolerance, double upperTolerance,
                             double[][] gearRatios, String SubsystemName) {
        this.type = type;

        motor = new TalonFX(id);
        motorConfig = new TalonFXConfiguration();

        this.velocity = velocity;
        this.acceleration = acceleration;
        this.jerk = jerk;

        dynamicMotionMagicVoltage = new DynamicMotionMagicVoltage(0, this.velocity, this.acceleration)
                .withJerk(this.jerk).withSlot(0).withEnableFOC(true);
        motionMagicVelocityVoltage = new MotionMagicVelocityVoltage(0).withSlot(0)
                .withEnableFOC(true);
        positionVoltage = new PositionVoltage(0).withSlot(0)
                .withEnableFOC(true);
        velocityVoltage = new VelocityVoltage(0).withSlot(0)
                .withEnableFOC(true);

        this.lowerTolerance = lowerTolerance;
        this.upperTolerance = upperTolerance;

        for (double[] ratio : gearRatios) {
            this.gearRatio *= (ratio[1] / ratio[0]);
        }

        /* Subsystem Logging on AdvantageKit */
        networkTable = NetworkTableInstance.getDefault().getTable("Subsystems/" + SubsystemName);

        poseData = networkTable.getDoubleTopic("Position").publish();
        velocityData = networkTable.getDoubleTopic("Velocity").publish();
        voltageData = networkTable.getDoubleTopic("Voltage").publish();
        supplyCurrentData = networkTable.getDoubleTopic("SupplyCurrent").publish();
        statorCurrentData = networkTable.getDoubleTopic("StatorCurrent").publish();
        tempData = networkTable.getDoubleTopic("Temp").publish();

        this.name = SubsystemName;
    }

    public String getName() {
        return name;
    }

    //Configurations
    public void configureMotor(boolean isInverted, boolean isBrakeMode,
                               double supplyCurrentLimit, double statorCurrentLimit,
                               Slot0Configs slot0Configs) {
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

        motorConfig.MotorOutput.ControlTimesyncFreqHz = 50;

        motor.optimizeBusUtilization(50);

        motor.getConfigurator().apply(motorConfig);
        motor.setPosition(0);
    }


    public void configureLinearMech(double drumCircumference, double motorMinRotation, double motorMaxRotation) {
        this.drumCircumference = drumCircumference;
        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = motorMinRotation;
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = motorMaxRotation;

        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        motor.getConfigurator().apply(motorConfig);
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

    public void configureFollowerMotor(int followerMotorId, boolean opposeMasterDirection) {
        followerMotor = new TalonFX(followerMotorId);
        follower = new Follower(motor.getDeviceID(),
                opposeMasterDirection ? MotorAlignmentValue.Opposed : MotorAlignmentValue.Aligned);
        followerMotor.setControl(follower);
    }

    public void configureSecondaryMotor(int motorID, double secondaryVelocity,
                                        double secondaryAcceleration, double secondaryJerk,
                                        boolean isInverted, boolean isBrakeMode,
                                        double supplyCurrentLimit, double statorCurrentLimit,
                                        Slot0Configs slot0Configs) {
        secondaryMotor = new TalonFX(motorID);
        secondaryMotorConfig = new TalonFXConfiguration();
        secondaryMotionMagicVelocityVoltage = new MotionMagicVelocityVoltage(0)
                .withSlot(0).withEnableFOC(true);

        secondaryMotorConfig.MotorOutput.Inverted =
                isInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        secondaryMotorConfig.MotorOutput.NeutralMode = isBrakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        secondaryMotorConfig.CurrentLimits.SupplyCurrentLimit = supplyCurrentLimit;
        secondaryMotorConfig.CurrentLimits.StatorCurrentLimit = statorCurrentLimit;
        secondaryMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        secondaryMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        secondaryMotorConfig.Slot0 = slot0Configs;

//        secondaryMotorConfig.MotionMagic.MotionMagicCruiseVelocity = secondaryVelocity;
//        secondaryMotorConfig.MotionMagic.MotionMagicAcceleration = secondaryAcceleration;
//        secondaryMotorConfig.MotionMagic.MotionMagicJerk = secondaryJerk;

        secondaryMotor.getConfigurator().apply(secondaryMotorConfig);
        secondaryMotor.setPosition(0);
    }

    public void configureEncoder(int encoderId, String canbus, double magnetOffset,
                                 double sensorToMechRatio, double motorToSensorRatio, boolean isCCWPositive) {
        encoder = new CANcoder(encoderId);
        encoderConfig = new CANcoderConfiguration();

        encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
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

    public void configureSometimesEncoder(int encoderId, String canbus, double magnetOffset,
                                          double sensorToMechRatio, double motorToSensorRatio,
                                          boolean isCCWPositive, double absoluteDiscontinuityPoint) {
        sometimesEncoder = new CANcoder(encoderId);
        sometimesEncoderConfig = new CANcoderConfiguration();

        sometimesEncoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = absoluteDiscontinuityPoint;
        sometimesEncoderConfig.MagnetSensor.SensorDirection = isCCWPositive ? SensorDirectionValue.CounterClockwise_Positive
                : SensorDirectionValue.Clockwise_Positive;

        sometimesEncoderConfig.MagnetSensor.MagnetOffset = magnetOffset;
        sometimesEncoder.getConfigurator().apply(sometimesEncoderConfig);

        this.motorToSensorRatio = motorToSensorRatio;

        motor.setPosition(sometimesEncoder.getAbsolutePosition().getValueAsDouble() * motorToSensorRatio);
    }

    public void configureCustomFF() {
        simpleMotorFeedforward = new SimpleMotorFeedforward(slot0Configs.kS,
                slot0Configs.kV, slot0Configs.kA);
    }

    public double getFeedForward(double velocity) {
        return simpleMotorFeedforward.calculate(velocity);
    }

    public void setPercent(double percent) {
        followLastMechProfile = false;
        if (percent > 1) percent /= 100;
        motor.set(percent);
    }

    public void setSecondaryPercent(double percent) {
        followLastMechProfile = false;
        if (percent > 1) percent /= 100;
        secondaryMotor.set(percent);
    }

    public void setVoltage(double output) {
        followLastMechProfile = false;
        motor.setVoltage(output);
    }

    public void setSecondaryVoltage(double output) {
        followLastMechProfile = false;
        secondaryMotor.setVoltage(output);
    }

    public void setVelocity(double rps) {
        this.goal = rps;
        followLastMechProfile = false;
        if (rps == 0) setPercent(0);
        else motor.setControl(motionMagicVelocityVoltage.withVelocity(rps));
    }

    public void setSecondaryVelocity(double rps) {
        this.secondaryGoal = rps;
        followLastMechProfile = false;
        if (rps == 0) setPercent(0);
        else secondaryMotor.setControl(secondaryMotionMagicVelocityVoltage.withVelocity(rps));
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
        motor.setControl(dynamicMotionMagicVoltage.withPosition(goalRotations));

//        dynamicMotionMagicVoltage.Velocity = this.velocity;
//        dynamicMotionMagicVoltage.Acceleration = this.acceleration;
//        dynamicMotionMagicVoltage.Jerk = this.jerk;
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
        motor.setControl(dynamicMotionMagicVoltage.withPosition(goalRotations)
                .withFeedForward(feedforward));

//        dynamicMotionMagicVoltage.Velocity = this.velocity;
//        dynamicMotionMagicVoltage.Acceleration = this.acceleration;
//        dynamicMotionMagicVoltage.Jerk = this.jerk;
    }

    public void setPositionVoltage(double motorRotations, double feedforward) {
        motor.setControl(positionVoltage.withPosition(motorRotations).withFeedForward(feedforward));
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


        motor.setControl(dynamicMotionMagicVoltage.withPosition(goalRotations));
    }

    public void setConstraints(double velocity, double acceleration, double jerk) {
        dynamicMotionMagicVoltage.Velocity = velocity;
        dynamicMotionMagicVoltage.Acceleration = acceleration;
        dynamicMotionMagicVoltage.Jerk = jerk;
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

    public boolean isAboveSpeed() {
        if (type != Type.ROLLER) return false;
        return getMechVelocity() > goal - lowerTolerance;
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

    public double getGoal() {
        return goal;
    }

    public double getSecondaryGoal() {
        return secondaryGoal;
    }

    public void setFollowLastMechProfile(boolean followLastMechProfile) {
        this.followLastMechProfile = followLastMechProfile;
    }


    //Unit Conversions
    public double getDegrees() {
        return encoder == null ? motor.getRotorPosition().getValueAsDouble() * gearRatio * 360d
                : getEncoderDegrees();
    }

    /**
     * @return Gets the position of the actual motor of the mechanism.
     */

    public double getMotorRot() {
        return motor.getRotorPosition().getValueAsDouble();
    }

    /**
     * @return The rotation of the mechanism itself (Accounts for gear ratios and stuff)
     */
    public double getMechRot() {
        return motor.getRotorPosition().getValueAsDouble() * gearRatio;
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
        return motor.getRotorPosition().getValueAsDouble() * drumCircumference * gearRatio;
    }

    /**
     * @return a constantly updating value for the height of the elevator as a double.
     */
    public DoubleSupplier getMechMeter() {
        if (type != Type.LINEAR) return () -> 0;
        return () -> motor.getRotorPosition().getValueAsDouble() * drumCircumference * gearRatio;
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
        return motor.getRotorVelocity().getValueAsDouble();
    }

    public double getSecondaryMotorVelocity() {
        return secondaryMotor.getRotorVelocity().getValueAsDouble();
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

    public TalonFX getFollowerMotor() {
        return followerMotor;
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

    @Override
    public void periodic() {
        if (changedOffset) {
            setPosition(goal);
            changedOffset = false;
        }


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

    public boolean hasSecondaryMotor() {
        return !(secondaryMotor == null);
    }

    public boolean isCommandRunning() {
        return isCommandRunning;
    }

    public void setCommandRunning(boolean commandRunning) {
        isCommandRunning = commandRunning;
    }

    public double getGearRatio() {
        return gearRatio;
    }
}