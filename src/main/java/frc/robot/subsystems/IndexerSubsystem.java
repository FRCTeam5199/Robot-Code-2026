package frc.robot.subsystems;

import frc.robot.constants.IntakeRollerConstants;
import frc.robot.subsystems.templates.TemplateSubsystem;
import frc.robot.utility.Type;

public class IndexerSubsystem extends TemplateSubsystem {
    private static IndexerSubsystem indexerSubsystem;

    private IndexerSubsystem() {
        super(Type.ROLLER, IntakeRollerConstants.INTAKE_ROLLER_MOTOR_ID,
                0, IntakeRollerConstants.INTAKE_ROLLER_ACCELERATION,
                IntakeRollerConstants.INTAKE_ROLLER_JERK,
                IntakeRollerConstants.INTAKE_ROLLER_LOWER_TOLERANCE,
                IntakeRollerConstants.INTAKE_ROLLER_UPPER_TOLERANCE,
                IntakeRollerConstants.INTAKE_ROLLER_GEAR_RATIO, "INTAKE_ROLLER");

        configureMotor(IntakeRollerConstants.INTAKE_ROLLER_INVERTED, IntakeRollerConstants.INTAKE_ROLLER_BRAKE,
                IntakeRollerConstants.INTAKE_ROLLER_SUPPLY_CURRENT_LIMIT,
                IntakeRollerConstants.INTAKE_ROLLER_STATOR_CURRENT_LIMIT,
                IntakeRollerConstants.INTAKE_ROLLER_SLOT0_CONFIGS);
    }

    public static IndexerSubsystem getInstance() {
        if (indexerSubsystem == null) {
            indexerSubsystem = new IndexerSubsystem();
        }
        return indexerSubsystem;
    }

    @Override
    public void periodic() {
        super.periodic();
//        System.out.println("Velocity: " + getMotorVelocity());
    }
}
