package frc.robot.subsystems;

import frc.robot.constants.IntakeRollerConstants;
import frc.robot.subsystems.templates.TemplateSubsystem;
import frc.robot.utility.Type;

public class IntakeRollerSubsystem extends TemplateSubsystem {
    private static IntakeRollerSubsystem intakeRollerSubsystem;

    private IntakeRollerSubsystem() {
        super(Type.ROLLER, IntakeRollerConstants.MOTOR_ID,
                0, IntakeRollerConstants.ACCELERATION,
                IntakeRollerConstants.JERK,
                IntakeRollerConstants.LOWER_TOLERANCE,
                IntakeRollerConstants.UPPER_TOLERANCE,
                IntakeRollerConstants.GEAR_RATIO, "Intake Roller", true, IntakeRollerConstants.canbus);

        configureMotor(IntakeRollerConstants.INVERTED, IntakeRollerConstants.BRAKE,
                IntakeRollerConstants.SUPPLY_CURRENT_LIMIT,
                IntakeRollerConstants.STATOR_CURRENT_LIMIT,
                IntakeRollerConstants.SLOT0_CONFIGS);
    }

    public static IntakeRollerSubsystem getInstance() {
        if (intakeRollerSubsystem == null) {
            intakeRollerSubsystem = new IntakeRollerSubsystem();
        }
        return intakeRollerSubsystem;
    }

    @Override
    public void periodic() {
        super.periodic();
        //    System.out.println("Velocity: " + getMotorVelocity());
        // System.out.println("Goal: " + getGoal());
    }
}
