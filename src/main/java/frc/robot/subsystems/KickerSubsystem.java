package frc.robot.subsystems;

import frc.robot.constants.KickerConstants;
import frc.robot.subsystems.templates.MotorGroup;
import frc.robot.subsystems.templates.TemplateSubsystem;
import frc.robot.utility.Type;

public class KickerSubsystem extends TemplateSubsystem {
    private static KickerSubsystem kickerSubsystem;

    private KickerSubsystem() {
        super("Indexer");

        MotorGroup motors = addMotorGroup(new MotorGroup(Type.ROLLER, KickerConstants.MOTOR_ID,
                0, KickerConstants.ACCELERATION,
                KickerConstants.JERK,
                KickerConstants.LOWER_TOLERANCE,
                KickerConstants.UPPER_TOLERANCE,
                KickerConstants.GEAR_RATIO, true, KickerConstants.canbus));

        motors.configureMotor(KickerConstants.INVERTED, KickerConstants.BRAKE,
                KickerConstants.SUPPLY_CURRENT_LIMIT,
                KickerConstants.STATOR_CURRENT_LIMIT,
                KickerConstants.SLOT0_CONFIGS, false);

    }

    public static KickerSubsystem getInstance() {
        if (kickerSubsystem == null) {
            kickerSubsystem = new KickerSubsystem();
        }
        return kickerSubsystem;
    }

    @Override
    public void periodic() {
        super.periodic();
    }
}
