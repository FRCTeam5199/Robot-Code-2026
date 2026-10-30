package frc.robot.subsystems;

import frc.robot.constants.IntakeLinearConstants;
import frc.robot.subsystems.templates.TemplateSubsystem;
import frc.robot.utility.Type;

public class IntakeLinearLeft extends TemplateSubsystem {
    private static IntakeLinearLeft intakePivotSubsystem;

    private IntakeLinearLeft() {
        super(Type.LINEAR, IntakeLinearConstants.LEFT_MOTOR_ID, IntakeLinearConstants.VELOCITY,
                IntakeLinearConstants.ACCELERATION, IntakeLinearConstants.JERK,
                IntakeLinearConstants.LOWER_TOLERANCE,
                IntakeLinearConstants.UPPER_TOLERANCE,
                IntakeLinearConstants.GEAR_RATIOA, "Intake Linear Left", true, IntakeLinearConstants.CAN_BUS);

        configureMotor(!IntakeLinearConstants.INVERTED, IntakeLinearConstants.BRAKE,
                IntakeLinearConstants.SUPPLY_CURRENT_LIMIT,
                IntakeLinearConstants.STATOR_CURRENT_LIMIT,
                IntakeLinearConstants.SLOT0_CONFIGS, false);

        configureLinearMech(2 * Math.PI, IntakeLinearConstants.MIN, IntakeLinearConstants.MAX);

    }

    public static IntakeLinearLeft getInstance() {
        if (intakePivotSubsystem == null) {
            intakePivotSubsystem = new IntakeLinearLeft();
        }
        return intakePivotSubsystem;
    }

    @Override
    public void periodic() {
        super.periodic();
        // System.out.println("poseLeft: " + getGoal());
    }
}
