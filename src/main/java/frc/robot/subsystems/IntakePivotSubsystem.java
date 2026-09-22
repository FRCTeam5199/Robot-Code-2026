package frc.robot.subsystems;

import frc.robot.constants.IntakePivotConstants;
import frc.robot.subsystems.templates.TemplateSubsystem;
import frc.robot.utility.SubsystemType;

public class IntakePivotSubsystem extends TemplateSubsystem {
    private static IntakePivotSubsystem intakePivotSubsystem;

    private IntakePivotSubsystem() {
        super(SubsystemType.PIVOT, IntakePivotConstants.MOTOR_ID, IntakePivotConstants.CANBUS, IntakePivotConstants.VELOCITY,
                IntakePivotConstants.ACCELERATION, IntakePivotConstants.JERK,
                IntakePivotConstants.LOWER_TOLERANCE,
                IntakePivotConstants.UPPER_TOLERANCE,
                IntakePivotConstants.GEAR_RATIOA, "Intake Pivot", true);

        configureMotor(IntakePivotConstants.INVERTED, IntakePivotConstants.BRAKE,
                IntakePivotConstants.SUPPLY_CURRENT_LIMIT,
                IntakePivotConstants.STATOR_CURRENT_LIMIT,
                IntakePivotConstants.SLOT0_CONFIGS, false);
    }

    public static IntakePivotSubsystem getInstance() {
        if (intakePivotSubsystem == null) {
            intakePivotSubsystem = new IntakePivotSubsystem();
        }
        return intakePivotSubsystem;
    }

    public void periodic() {
        super.periodic();
    }
}
