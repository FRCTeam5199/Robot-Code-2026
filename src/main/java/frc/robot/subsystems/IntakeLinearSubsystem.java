package frc.robot.subsystems;

import frc.robot.constants.IntakeLinearConstants;
import frc.robot.subsystems.templates.TemplateSubsystem;
import frc.robot.utility.Type;

public class IntakeLinearSubsystem extends TemplateSubsystem {
    private static IntakeLinearSubsystem intakePivotSubsystem;

    private IntakeLinearSubsystem() {
        super(Type.LINEAR, IntakeLinearConstants.MOTOR_ID, IntakeLinearConstants.VELOCITY,
                IntakeLinearConstants.ACCELERATION, IntakeLinearConstants.JERK,
                IntakeLinearConstants.LOWER_TOLERANCE,
                IntakeLinearConstants.UPPER_TOLERANCE,
                IntakeLinearConstants.GEAR_RATIOA, "Intake Linear", true, IntakeLinearConstants.CAN_BUS);

        configureMotor(IntakeLinearConstants.INVERTED, IntakeLinearConstants.BRAKE,
                IntakeLinearConstants.SUPPLY_CURRENT_LIMIT,
                IntakeLinearConstants.STATOR_CURRENT_LIMIT,
                IntakeLinearConstants.SLOT0_CONFIGS, false);

        configureFollowerMotor(IntakeLinearConstants.FOLLOWER_MOTOR_ID,
                IntakeLinearConstants.FOLLOWER_INVERTED, IntakeLinearConstants.CAN_BUS, true);

    }

    public static IntakeLinearSubsystem getInstance() {
        if (intakePivotSubsystem == null) {
            intakePivotSubsystem = new IntakeLinearSubsystem();
        }
        return intakePivotSubsystem;
    }

    public void periodic() {
        super.periodic();
    }
}
