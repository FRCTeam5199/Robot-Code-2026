package frc.robot.subsystems.templates;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
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

import org.wpilib.math.controller.SimpleMotorFeedforward;
import org.wpilib.networktables.DoublePublisher;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.command2.SubsystemBase;
import frc.robot.utility.Type;

public class TemplateSubsystem extends SubsystemBase {
    public StatusSignal<Angle> positionStatusSignal;
    public StatusSignal<AngularVelocity> velocityStatusSignal;
    public StatusSignal<Angle> followerPositionStatusSignal;
    public StatusSignal<AngularVelocity> followerVelocityStatusSignal;
    private NetworkTable networkTable;
    private DoublePublisher poseData;
    private DoublePublisher velocityData;
    private DoublePublisher voltageData;
    private DoublePublisher supplyCurrentData;
    private DoublePublisher statorCurrentData;
    private DoublePublisher tempData;
    private TalonFX motor;
    private CANBus canBus;
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
    private MotionMagicVoltage motionMagicVoltage;
    private MotionMagicVelocityVoltage motionMagicVelocityVoltage;
    private MotionMagicVelocityVoltage secondaryMotionMagicVelocityVoltage;
    private PositionVoltage positionVoltage;
    private VelocityVoltage velocityVoltage;
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
                             double[][] gearRatios, String SubsystemName, boolean enableFoc) {
        this.type = type;

        motor = new TalonFX(id);
        motorConfig = new TalonFXConfiguration();

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

        /* Subsystem Logging on AdvantageKit */
//        networkTable = NetworkTableInstance.getDefault().getTable("Subsystems/" + SubsystemName);
//
//        poseData = networkTable.getDoubleTopic("Position").publish();
//        velocityData = networkTable.getDoubleTopic("Velocity").publish();
//        voltageData = networkTable.getDoubleTopic("Voltage").publish();
//        supplyCurrentData = networkTable.getDoubleTopic("SupplyCurrent").publish();
//        statorCurrentData = networkTable.getDoubleTopic("StatorCurrent").publish();
//        tempData = networkTable.getDoubleTopic("Temp").publish();

        this.name = SubsystemName;
    }

    public TemplateSubsystem(Type type, int id, double velocity, double acceleration, double jerk,
                             double lowerTolerance, double upperTolerance,
                             double[][] gearRatios, String SubsystemName, boolean enableFoc, CANBus canbus) {
        this.type = type;

//        motor = new TalonFX(id, "CANBUS");
        motor = new TalonFX(id, canbus);
        motorConfig = new TalonFXConfiguration();

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

        /* Subsystem Logging on AdvantageKit */
//        networkTable = NetworkTableInstance.getDefault().getTable("Subsystems/" + SubsystemName);
//
//        poseData = networkTable.getDoubleTopic("Position").publish();
//        velocityData = networkTable.getDoubleTopic("Velocity").publish();
//        voltageData = networkTable.getDoubleTopic("Voltage").publish();
//        supplyCurrentData = networkTable.getDoubleTopic("SupplyCurrent").publish();
//        statorCurrentData = networkTable.getDoubleTopic("StatorCurrent").publish();
//        tempData = networkTable.getDoubleTopic("Temp").publish();

        this.name = SubsystemName;
    }

    public String getName() {
        return name;
    }

    //Configurations
    public void configureMotor(boolean isInverted, boolean isBrakeMode,
                               double supplyCurrentLimit, double statorCurrentLimit,
                               Slot0Configs slot0Configs, boolean faster) {
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

        positionStatusSignal = motor.getRotorPosition();
        velocityStatusSignal = motor.getRotorVelocity();

        if (faster) {
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

    public void configureFollowerMotor(int followerMotorId, boolean opposeMasterDirection, CANBus canbus) {
        followerMotor = new TalonFX(followerMotorId, canbus);

        motor.getDutyCycle().setUpdateFrequency(100);
        motor.getMotorVoltage().setUpdateFrequency(100);
        motor.getTorqueCurrent().setUpdateFrequency(100);

        follower = new Follower(motor.getDeviceID(),
                opposeMasterDirection ? MotorAlignmentValue.Opposed : MotorAlignmentValue.Aligned);
        followerMotor.setControl(follower);
    }

    public void configureFollowerMotor(int followerMotorId, boolean opposeMasterDirection, CANBus canbus, boolean special) {
        followerMotor = new TalonFX(followerMotorId, canbus);

        motor.getDutyCycle().setUpdateFrequency(100);
        motor.getMotorVoltage().setUpdateFrequency(100);
        motor.getTorqueCurrent().setUpdateFrequency(100);

        followerMotor.getRotorVelocity().setUpdateFrequency(50);
        followerMotor.optimizeBusUtilization();

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

    public void setPercent(double percent) {
        followLastMechProfile = false;
        if (percent > 1) percent /= 100d;
        motor.set(percent);
    }

    public void setSecondaryPercent(double percent) {
        followLastMechProfile = false;
        if (percent > 1) percent /= 100d;
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
        else motor.setControl(velocityVoltage.withVelocity(rps + offset));
    }

    //TODO: change to velocityVoltage
    public void setSecondaryVelocity(double rps) {
        this.secondaryGoal = rps;
        followLastMechProfile = false;
        if (rps == 0) setPercent(0);
        else secondaryMotor.setControl(secondaryMotionMagicVelocityVoltage.withVelocity(rps + offset));
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

    public double getSecondaryGoal() {
        return secondaryGoal;
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

        BaseStatusSignal.refreshAll(positionStatusSignal, velocityStatusSignal);

//        if (followerMotor != null) {
//            followerPositionStatusSignal.refresh();
//            followerVelocityStatusSignal.refresh();
//        }


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