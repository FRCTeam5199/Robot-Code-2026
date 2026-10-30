package frc.robot.subsystems;

import frc.robot.constants.IntakeLinearConstants;
import frc.robot.subsystems.templates.TemplateSubsystem;
import frc.robot.utility.Type;

public class IntakeLinearRight extends TemplateSubsystem {
    private static IntakeLinearRight intakePivotSubsystem;

    private IntakeLinearRight() {
        super(Type.LINEAR, IntakeLinearConstants.RIGHT_MOTOR_ID, IntakeLinearConstants.VELOCITY,
                IntakeLinearConstants.ACCELERATION, IntakeLinearConstants.JERK,
                IntakeLinearConstants.LOWER_TOLERANCE,
                IntakeLinearConstants.UPPER_TOLERANCE,
                IntakeLinearConstants.GEAR_RATIOA, "Intake Linear Right", true, IntakeLinearConstants.CAN_BUS);

        configureMotor(IntakeLinearConstants.INVERTED, IntakeLinearConstants.BRAKE,
                IntakeLinearConstants.SUPPLY_CURRENT_LIMIT,
                IntakeLinearConstants.STATOR_CURRENT_LIMIT,
                IntakeLinearConstants.SLOT0_CONFIGS, false);


        configureLinearMech(2 * Math.PI, IntakeLinearConstants.MIN, IntakeLinearConstants.MAX);

    }

    public static IntakeLinearRight getInstance() {
        if (intakePivotSubsystem == null) {
            intakePivotSubsystem = new IntakeLinearRight();
        }
        return intakePivotSubsystem;
    }

    @Override
    public void periodic() {
        super.periodic();
        // System.out.println("poseRight: " + getGoal());

    }
}
