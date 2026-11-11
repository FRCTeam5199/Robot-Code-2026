package frc.robot.subsystems.templates;


import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.subsystems.warnings.RollerLimitsWarning;
import frc.robot.units.MotorRotations;
import frc.robot.units.RotationalUnit;

public class RollerSubsystem extends TemplateSubsystem {

    protected void configureMotor(int motorID, boolean clockwise, double[][] speedGearRatios, CANBus CANBusObject,
								Slot0Configs slotConfig,
								RotationalUnit fwdLimit, RotationalUnit reverseLimit, 
								double statorLimit, double supplyLimit, 
								RotationalUnit velocityTolerance,
								RotationalUnit cruiseVelocity, RotationalUnit acceleration, RotationalUnit jerk){
        //
        if(fwdLimit != null) emitWarning(new RollerLimitsWarning("Roller passed in with non-null fwdLimit."));
        if(reverseLimit != null) emitWarning(new RollerLimitsWarning("Roller passed in with non-null reverseLimit."));

        super.configureMotor(motorID, clockwise, speedGearRatios, CANBusObject, slotConfig, fwdLimit, reverseLimit, statorLimit, supplyLimit, NeutralModeValue.Coast, new MotorRotations(0), velocityTolerance, cruiseVelocity, acceleration, jerk);
    }

    public void setPosition(RotationalUnit position){
        super.setPosition(position.asMotorRotations(this));
    }

    public void setVelocity(RotationalUnit velocity){
        super.setVelocity(velocity.asMotorRotations(this));
    }
}
