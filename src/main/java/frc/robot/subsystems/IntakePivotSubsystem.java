package frc.robot.subsystems;

import frc.robot.constants.IntakePivotConstants;
import frc.robot.subsystems.templates.TemplateSubsystem;
import frc.robot.utility.Type;

public class IntakePivotSubsystem extends TemplateSubsystem {
    private static IntakePivotSubsystem intakePivotSubsystem;
    public double goal = 70;

    private IntakePivotSubsystem() {
        super(Type.PIVOT, IntakePivotConstants.MOTOR_ID, IntakePivotConstants.VELOCITY,
                IntakePivotConstants.ACCELERATION, IntakePivotConstants.JERK,
                IntakePivotConstants.LOWER_TOLERANCE,
                IntakePivotConstants.UPPER_TOLERANCE,
                IntakePivotConstants.GEAR_RATIOA, "Intake Pivot");

        configureMotor(IntakePivotConstants.INVERTED, IntakePivotConstants.BRAKE,
                IntakePivotConstants.SUPPLY_CURRENT_LIMIT,
                IntakePivotConstants.STATOR_CURRENT_LIMIT,
                IntakePivotConstants.SLOT0_CONFIGS);

        configureEncoder(IntakePivotConstants.ENCODER_ID,
                "rio", IntakePivotConstants.MAGNET_OFFSET,
                IntakePivotConstants.INTAKE_SENSOR_MECH_RATIO,
                IntakePivotConstants.GEAR_RATIO,
                IntakePivotConstants.ENCODER_DIRECTION);

        configurePivot(IntakePivotConstants.MIN,
                IntakePivotConstants.MAX);
    }

    public static IntakePivotSubsystem getInstance() {
        if (intakePivotSubsystem == null) {
            intakePivotSubsystem = new IntakePivotSubsystem();
        }
        return intakePivotSubsystem;
    }

    public void periodic() {
        super.periodic();
        //    System.out.println("Degrees: " + getDegrees());
        //    System.out.println("Goal: " + getGoal());
        // System.out.println("Is at Goal: "+ isMechAtGoal(false));
    }
}
