package first.robot.subsystems.templates;

import java.util.HashSet;
import java.util.Set;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;


import first.robot.constants.SetPoints;
import first.robot.subsystems.warnings.IllegalCurrentLimitWarning;
import first.robot.subsystems.warnings.SelfFollowingWarning;
import first.robot.subsystems.warnings.Warning;
import first.robot.units.MotorRotations;
import first.robot.units.RotationalUnit;

import com.ctre.phoenix6.controls.VelocityVoltage;
import org.wpilib.command2.SubsystemBase;

public abstract class TemplateSubsystem extends SubsystemBase {
	private Set<Class<Warning>> suppressedWarnings = new HashSet<>();

	// setup constants
	protected TalonFX motor;
	protected TalonFX follower = null; // may not be set
	private double speedGearRatio;
	private MotorRotations positionTolerance;
	private MotorRotations velocityTolerance;

	protected SetPoints positionSetPoints;
	protected SetPoints velocitySetPoints;

	// motion
	private MotorRotations goalPosition = new MotorRotations(0);
	private MotorRotations goalVelocity = new MotorRotations(0);
	private final MotionMagicVoltage motionMagicRequest;
	private final VelocityVoltage velocityRequest;

	public TemplateSubsystem(){
		motionMagicRequest = new MotionMagicVoltage(0).withEnableFOC(true).withSlot(0);
		velocityRequest = new VelocityVoltage(0).withEnableFOC(true).withSlot(0);
	}

	public double getSpeedGearRatio(){
		return speedGearRatio;
	}

	public double getTorqueGearRatio(){
		return 1/speedGearRatio;
	}

	protected void configureMotor(int motorID, boolean clockwise, double[][] speedGearRatios, CANBus CANBusObject,
								Slot0Configs slotConfig,
								RotationalUnit fwdLimit, RotationalUnit reverseLimit, 
								double statorLimit, double supplyLimit, 
								NeutralModeValue neutralMode,
								RotationalUnit positionTolerance, RotationalUnit velocityTolerance,
								RotationalUnit cruiseVelocity, RotationalUnit acceleration, RotationalUnit jerk){ // motion magic
		//
		this.speedGearRatio = 0;
		for(double[] ratio : speedGearRatios){
			speedGearRatio += ratio[1] / ratio[0]; 
		}

		TalonFXConfiguration config = new TalonFXConfiguration();

		config.MotorOutput.Inverted = clockwise ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
		config.MotorOutput.NeutralMode = neutralMode;
		
		if(fwdLimit != null){
			config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
			config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = fwdLimit.asMotorRotations(this).magnitude;
		}

		if(reverseLimit != null){
			config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
			config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = reverseLimit.asMotorRotations(this).magnitude;
		}

		config.CurrentLimits.StatorCurrentLimitEnable = true;
		config.CurrentLimits.SupplyCurrentLimitEnable = true;

		config.CurrentLimits.StatorCurrentLimit = statorLimit;
		config.CurrentLimits.SupplyCurrentLimit = supplyLimit;

		config.MotionMagic.MotionMagicCruiseVelocity = cruiseVelocity.asMotorRotations(this).magnitude;
		config.MotionMagic.MotionMagicAcceleration = acceleration.asMotorRotations(this).magnitude;
		config.MotionMagic.MotionMagicJerk = jerk.asMotorRotations(this).magnitude;

		config.Slot0 = slotConfig;

		motor = new TalonFX(motorID, CANBusObject);
		motor.getConfigurator().apply(config);
		motor.setPosition(0); // reset internal pos

		this.positionTolerance = positionTolerance.asMotorRotations(this);
		this.velocityTolerance = positionTolerance.asMotorRotations(this);

		if(statorLimit <= 0) emitWarning(new IllegalCurrentLimitWarning("Stator limit was passed in as `"+statorLimit+"` which is equal-to or less-than zero (illegal)."));
		if(supplyLimit <= 0) emitWarning(new IllegalCurrentLimitWarning("Supply limit was passed in as `"+statorLimit+"` which is equal-to or less-than zero (illegal)."));
	}

	public void configureMotorFollower(int motorID, int followerID, boolean invertFollower){
		if(motorID == followerID){
			emitWarning(new SelfFollowingWarning("Motor `"+followerID+"` has been set to follow itself..."));
		} 
		follower = new TalonFX(followerID, null);
		follower.setControl(new Follower(motorID, invertFollower ? MotorAlignmentValue.Opposed : MotorAlignmentValue.Aligned));
	}

	// ======================================================
	//
	// controls
	//
	// ======================================================

	public void setPosition(MotorRotations position){	
		motor.setControl(motionMagicRequest.withPosition(position.magnitude));
		goalPosition = position;
	}

	public void setVelocity(MotorRotations velocity) {
		motor.setControl(velocityRequest.withVelocity(velocity.magnitude));
		goalVelocity = velocity;
	}

	public void setPercentage(double percent){
		this.motor.setThrottle(percent);
	}

	public void setVoltage(double voltage){
		motor.setVoltage(voltage); // internally compensates for battery sag apparently
	}

	// ======================================================
	//
	// misc
	//
	// ======================================================

	public boolean isMechAtGoal(boolean usePosition, boolean useVelocity) {
		double current = motor.getPosition().getValueAsDouble();

		return ((Math.abs(current - goalPosition.magnitude) < positionTolerance.magnitude) || !usePosition)
			&& ((Math.abs(motor.getVelocity().getValueAsDouble() - goalVelocity.magnitude) < velocityTolerance.magnitude) || !useVelocity);
	}

	public void suppressWarning(Class<Warning> warning){
		this.suppressedWarnings.add(warning);
	}

	public void emitWarning(Warning warning){
		if(this.suppressedWarnings.contains(warning.getClass())) return;
		System.err.println("[WARNING "+getClass().getName()+"] "+warning.message);
	}
}
