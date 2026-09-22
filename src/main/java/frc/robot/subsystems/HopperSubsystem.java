package frc.robot.subsystems;

import frc.robot.constants.HopperConstants;
import frc.robot.subsystems.templates.TemplateSubsystem;
import frc.robot.utility.SubsystemType;

public class HopperSubsystem extends TemplateSubsystem {
    private static HopperSubsystem hopperSubsystem;

    private HopperSubsystem() {
        super(SubsystemType.ROLLER, HopperConstants.MOTOR_ID, HopperConstants.CANBUS,
                0, HopperConstants.ACCELERATION,
                HopperConstants.JERK,
                HopperConstants.LOWER_TOLERANCE,
                HopperConstants.UPPER_TOLERANCE,
                HopperConstants.GEAR_RATIO, "Hopper", true);

        configureMotor(HopperConstants.INVERTED, HopperConstants.BRAKE,
                HopperConstants.SUPPLY_CURRENT_LIMIT,
                HopperConstants.STATOR_CURRENT_LIMIT,
                HopperConstants.SLOT0_CONFIGS, false);
    }

    public static HopperSubsystem getInstance() {
        if (hopperSubsystem == null) {
            hopperSubsystem = new HopperSubsystem();
        }
        return hopperSubsystem;
    }

    @Override
    public void periodic() {
        super.periodic();
    }
}