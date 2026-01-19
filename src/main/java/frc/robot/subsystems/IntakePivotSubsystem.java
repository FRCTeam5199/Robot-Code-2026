package frc.robot.subsystems;

import frc.robot.generated.IntakePivotConstants;
import frc.robot.subsystems.templates.TemplateSubsystem;
import frc.robot.utility.Type;

public class IntakePivotSubsystem extends TemplateSubsystem {
    private static IntakePivotSubsystem intakePivotSubsystem;
    public static IntakePivotSubsystem getInstance() {
        if (intakePivotSubsystem == null) {
            intakePivotSubsystem = new IntakePivotSubsystem();
        }
        return intakePivotSubsystem;
    }

    private IntakePivotSubsystem() {
        super(Type.PIVOT, IntakePivotConstants.INTAKE_PIVOT_MOTOR_ID, IntakePivotConstants.INTAKE_PIVOT_VELOCITY,
                IntakePivotConstants.INTAKE_PIVOT_ACCELERATION, IntakePivotConstants.INTAKE_PIVOT_JERK,
                IntakePivotConstants.INTAKE_PIVOT_LOWER_TOLERANCE,
                IntakePivotConstants.INTAKE_PIVOT_UPPER_TOLERANCE,
                IntakePivotConstants.INTAKE_PIVOT_GEAR_RATIO, "INTAKE_PIVOT");

        configureMotor(IntakePivotConstants.INTAKE_PIVOT_INVERTED, IntakePivotConstants.INTAKE_PIVOT_BRAKE,
                IntakePivotConstants.INTAKE_PIVOT_SUPPLY_CURRENT_LIMIT,
                IntakePivotConstants.INTAKE_PIVOT_STATOR_CURRENT_LIMIT,
                IntakePivotConstants.INTAKE_PIVOT_SLOT0_CONFIGS);

        configurePivot(IntakePivotConstants.INTAKE_PIVOT_MIN_ROTATIONS,
                IntakePivotConstants.INTAKE_PIVOT_MAX_ROTATIONS);
    }

    @Override
    public void periodic() {
        super.periodic();
        System.out.println("Degrees: " + getDegrees());
        //-1.2, -9.35
    }
}
