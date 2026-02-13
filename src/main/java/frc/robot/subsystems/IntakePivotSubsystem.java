package frc.robot.subsystems;

import frc.robot.constants.IntakePivotConstants;
import frc.robot.subsystems.templates.TemplateSubsystem;
import frc.robot.utility.Type;

public class IntakePivotSubsystem extends TemplateSubsystem {
    private static IntakePivotSubsystem intakePivotSubsystem;
    public double goal = 70;

    private IntakePivotSubsystem() {
        super(Type.PIVOT, IntakePivotConstants.INTAKE_PIVOT_MOTOR_ID, IntakePivotConstants.INTAKE_PIVOT_VELOCITY_OUT,
                IntakePivotConstants.INTAKE_PIVOT_ACCELERATION_OUT, IntakePivotConstants.INTAKE_PIVOT_JERK_OUT,
                IntakePivotConstants.INTAKE_PIVOT_LOWER_TOLERANCE,
                IntakePivotConstants.INTAKE_PIVOT_UPPER_TOLERANCE,
                IntakePivotConstants.INTAKE_PIVOT_GEAR_RATIOA, "Intake Pivot");

        configureMotor(IntakePivotConstants.INTAKE_PIVOT_INVERTED, IntakePivotConstants.INTAKE_PIVOT_BRAKE,
                IntakePivotConstants.INTAKE_PIVOT_SUPPLY_CURRENT_LIMIT,
                IntakePivotConstants.INTAKE_PIVOT_STATOR_CURRENT_LIMIT,
                IntakePivotConstants.INTAKE_PIVOT_SLOT0_CONFIGS);

        configureEncoder(IntakePivotConstants.INTAKE_PIVOT_ENCODER_ID,
                "rio", IntakePivotConstants.INTAKE_PIVOT_MAGNET_OFFSET,
                IntakePivotConstants.INTAKE_SENSOR_MECH_RATIO,
                IntakePivotConstants.INTAKE_PIVOT_GEAR_RATIO,
                IntakePivotConstants.INTAKE_PIVOT_ENCODER_DIRECTION);

        configurePivot(IntakePivotConstants.INTAKE_PIVOT_MIN,
                IntakePivotConstants.INTAKE_PIVOT_MAX);
    }

    public static IntakePivotSubsystem getInstance() {
        if (intakePivotSubsystem == null) {
            intakePivotSubsystem = new IntakePivotSubsystem();
        }
        return intakePivotSubsystem;
    }

    public void periodic() {
        super.periodic();
//        System.out.println("Degrees: " + getDegrees());
//        System.out.println("Goal: " + getGoal());
    }
}
