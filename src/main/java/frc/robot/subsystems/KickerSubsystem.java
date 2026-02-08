package frc.robot.subsystems;

import frc.robot.constants.KickerConstants;
import frc.robot.subsystems.templates.TemplateSubsystem;
import frc.robot.utility.Type;

public class KickerSubsystem extends TemplateSubsystem {
    private static KickerSubsystem kickerSubsystem;

    private KickerSubsystem() {
        super(Type.ROLLER, KickerConstants.KICKER_MOTOR_ID,
                0, KickerConstants.KICKER_ACCELERATION,
                KickerConstants.KICKER_JERK,
                KickerConstants.KICKER_LOWER_TOLERANCE,
                KickerConstants.KICKER_UPPER_TOLERANCE,
                KickerConstants.KICKER_GEAR_RATIO, "Kicker");

        configureMotor(KickerConstants.KICKER_INVERTED, KickerConstants.KICKER_BRAKE,
                KickerConstants.KICKER_SUPPLY_CURRENT_LIMIT,
                KickerConstants.KICKER_STATOR_CURRENT_LIMIT,
                KickerConstants.KICKER_SLOT0_CONFIGS);
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
        System.out.println("Kicker Velocity: " + getMotorVelocity());
        // System.out.println("Kicker is at goal: " + isMechAtGoal(true));
    }
}
