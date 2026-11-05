package frc.robot.subsystems;

import frc.robot.constants.IntakeLinearConstants;
import frc.robot.subsystems.templates.MotorGroup;
import frc.robot.subsystems.templates.TemplateSubsystem;
import frc.robot.utility.Type;

public class IntakeLinearSubsystem extends TemplateSubsystem {
    private static IntakeLinearSubsystem intakePivotSubsystem;

    private IntakeLinearSubsystem() {
        super("Intake Linear");

        MotorGroup motors = addMotorGroup(new MotorGroup(Type.LINEAR, IntakeLinearConstants.MOTOR_ID, IntakeLinearConstants.VELOCITY,
                IntakeLinearConstants.ACCELERATION, IntakeLinearConstants.JERK,
                IntakeLinearConstants.LOWER_TOLERANCE,
                IntakeLinearConstants.UPPER_TOLERANCE,
                IntakeLinearConstants.GEAR_RATIOA, true, IntakeLinearConstants.CAN_BUS));

        motors.configureMotor(IntakeLinearConstants.INVERTED, IntakeLinearConstants.BRAKE,
                IntakeLinearConstants.SUPPLY_CURRENT_LIMIT,
                IntakeLinearConstants.STATOR_CURRENT_LIMIT,
                IntakeLinearConstants.SLOT0_CONFIGS, false);

        motors.addFollower(IntakeLinearConstants.FOLLOWER_MOTOR_ID, IntakeLinearConstants.FOLLOWER_INVERTED);

        motors.configureLinearMech(2 * Math.PI, IntakeLinearConstants.MIN, IntakeLinearConstants.MAX);

    }

    public static IntakeLinearSubsystem getInstance() {
        if (intakePivotSubsystem == null) {
            intakePivotSubsystem = new IntakeLinearSubsystem();
        }
        return intakePivotSubsystem;
    }

    @Override
    public void periodic() {
        super.periodic();
        // System.out.println("pose: " + getMechM());
    }
}
