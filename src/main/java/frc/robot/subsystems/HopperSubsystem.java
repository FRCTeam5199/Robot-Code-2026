package frc.robot.subsystems;

import frc.robot.constants.HopperConstants;
import frc.robot.subsystems.templates.TemplateSubsystem;
import frc.robot.utility.Type;

public class HopperSubsystem extends TemplateSubsystem {
    private static HopperSubsystem hopperSubsystem;
    public static HopperSubsystem getInstance() {
        if (hopperSubsystem == null) {
            hopperSubsystem = new HopperSubsystem();
        }
        return hopperSubsystem;
    }

    private HopperSubsystem() {
        super(Type.ROLLER, HopperConstants.HOPPER_MOTOR_ID,
                0, HopperConstants.HOPPER_ACCELERATION,
                HopperConstants.HOPPER_JERK,
                HopperConstants.HOPPER_LOWER_TOLERANCE,
                HopperConstants.HOPPER_UPPER_TOLERANCE,
                HopperConstants.HOPPER_GEAR_RATIO, "Hopper");

        configureMotor(HopperConstants.HOPPER_INVERTED, HopperConstants.HOPPER_BRAKE,
                HopperConstants.HOPPER_SUPPLY_CURRENT_LIMIT,
                HopperConstants.HOPPER_STATOR_CURRENT_LIMIT,
                HopperConstants.HOPPER_SLOT0_CONFIGS);
    }

    @Override
    public void periodic() {
        super.periodic();
//        System.out.println("Velocity: " + getMotorVelocity());
    }
}