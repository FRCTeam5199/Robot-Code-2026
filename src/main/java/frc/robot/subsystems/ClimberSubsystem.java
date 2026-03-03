package frc.robot.subsystems;

import frc.robot.constants.ClimberConstants;
import frc.robot.subsystems.templates.TemplateSubsystem;
import frc.robot.utility.Type;

public class ClimberSubsystem extends TemplateSubsystem {
    private static ClimberSubsystem climberSubsystem;

    private ClimberSubsystem() {
        super(Type.LINEAR, ClimberConstants.MOTOR_ID, ClimberConstants.VELOCITY,
                ClimberConstants.ACCELERATION, ClimberConstants.JERK,
                ClimberConstants.LOWER_TOLERANCE,
                ClimberConstants.UPPER_TOLERANCE,
                ClimberConstants.GEAR_RATIO, "Climber");

        configureMotor(ClimberConstants.INVERTED, ClimberConstants.BRAKE,
                ClimberConstants.SUPPLY_CURRENT_LIMIT,
                ClimberConstants.STATOR_CURRENT_LIMIT,
                ClimberConstants.SLOT0_CONFIGS);

        // configureLinearMech(ClimberConstants.DRUM_CIRCUMFERENCE, ClimberConstants.MIN,
        //         ClimberConstants.MAX);
    }

    public static ClimberSubsystem getInstance() {
        if (climberSubsystem == null) {
            climberSubsystem = new ClimberSubsystem();
        }
        return climberSubsystem;
    }

    public void periodic() {
        super.periodic();
    }
}
