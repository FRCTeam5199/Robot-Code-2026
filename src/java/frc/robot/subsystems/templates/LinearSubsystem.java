package frc.robot.subsystems.templates;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.units.LinearUnit;
import frc.robot.units.RotationalUnit;

public class LinearSubsystem extends TemplateSubsystem {
    private double metersPerMechRotation;

    protected void configureMotor(int motorID, boolean clockwise, double[][] speedGearRatios, double metersPerMechRotation, CANBus CANBusObject,
								Slot0Configs slotConfig,
								RotationalUnit fwdLimit, RotationalUnit reverseLimit, 
								double statorLimit, double supplyLimit, 
								NeutralModeValue neutralMode,
								RotationalUnit positionTolerance, RotationalUnit velocityTolerance,
								RotationalUnit cruiseVelocity, RotationalUnit acceleration, RotationalUnit jerk){
        //
        this.metersPerMechRotation = metersPerMechRotation;
        super.configureMotor(motorID, clockwise, speedGearRatios, CANBusObject, slotConfig.withGravityType(GravityTypeValue.Arm_Cosine), fwdLimit, reverseLimit, statorLimit, supplyLimit, neutralMode, positionTolerance, velocityTolerance, cruiseVelocity, acceleration, jerk);
    }

    public void setPosition(LinearUnit position){
        super.setPosition(position.asMotorRotations(this));
    }

    public void setVelocity(LinearUnit velocity){
        super.setVelocity(velocity.asMotorRotations(this));
    }

    public double getMetersPerMechRotation(){
        return this.metersPerMechRotation;
    }
}
