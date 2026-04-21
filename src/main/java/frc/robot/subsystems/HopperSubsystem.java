package frc.robot.subsystems;

import frc.robot.constants.HopperConstants;
import frc.robot.subsystems.templates.TemplateSubsystem;
import frc.robot.utility.Type;

public class HopperSubsystem extends TemplateSubsystem {
    private static HopperSubsystem hopperSubsystem;

    private HopperSubsystem() {
        super(Type.ROLLER, HopperConstants.MOTOR_ID,
                0, HopperConstants.ACCELERATION,
                HopperConstants.JERK,
                HopperConstants.LOWER_TOLERANCE,
                HopperConstants.UPPER_TOLERANCE,
                HopperConstants.GEAR_RATIO, "Hopper", true, HopperConstants.canbus);

        configureMotor(HopperConstants.INVERTED, HopperConstants.BRAKE,
                HopperConstants.SUPPLY_CURRENT_LIMIT,
                HopperConstants.STATOR_CURRENT_LIMIT,
                HopperConstants.SLOT0_CONFIGS, false);
    }

    private static HopperSubsystem getInstance() {
        if (hopperSubsystem == null) {
            hopperSubsystem = new HopperSubsystem();
        }
        return hopperSubsystem;
    }

    @Override
    public void periodic() {
        super.periodic();
        //    System.out.println("Velocity: " + getMotorVelocity());
//        System.out.println(isMechAtGoal(true));
    }
}