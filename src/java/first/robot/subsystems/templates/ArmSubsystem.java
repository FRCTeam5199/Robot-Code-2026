package first.robot.subsystems.templates;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.GravityTypeValue;

import first.robot.units.RotationalUnit;

public class ArmSubsystem extends TemplateSubsystem {

    protected void configureMotor(int motorID, boolean clockwise, double[][] speedGearRatios, CANBus CANBusObject,
								Slot0Configs slotConfig,
								RotationalUnit fwdLimit, RotationalUnit reverseLimit, 
								double statorLimit, double supplyLimit, 
								NeutralModeValue neutralMode,
								RotationalUnit positionTolerance, RotationalUnit velocityTolerance,
								RotationalUnit cruiseVelocity, RotationalUnit acceleration, RotationalUnit jerk){
        //
        super.configureMotor(motorID, clockwise, speedGearRatios, CANBusObject, slotConfig.withGravityType(GravityTypeValue.Arm_Cosine), fwdLimit, reverseLimit, statorLimit, supplyLimit, neutralMode, 
                             positionTolerance, velocityTolerance, cruiseVelocity, acceleration, jerk);
    }

    public void setPosition(RotationalUnit position){
        super.setPosition(position.asMotorRotations(this));
    }

    public void setVelocity(RotationalUnit velocity){
        super.setVelocity(velocity.asMotorRotations(this));
    }
}
