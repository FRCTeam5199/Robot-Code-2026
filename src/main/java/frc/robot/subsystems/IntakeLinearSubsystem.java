package frc.robot.subsystems;

import frc.robot.constants.IntakeLinearConstants;
import frc.robot.subsystems.templates.MotorGroup;
import frc.robot.subsystems.templates.TemplateSubsystem;
import frc.robot.utility.Type;

/**
 * The left and right sides are separate motor groups, because each side runs its own closed loop
 * and RobotCommands.intakeOut() backs each one off separately.
 */
public class IntakeLinearSubsystem extends TemplateSubsystem {
    private static IntakeLinearSubsystem intakeLinearSubsystem;

    private final MotorGroup right;
    private final MotorGroup left;

    private IntakeLinearSubsystem() {
        super("Intake Linear");

        right = createSide(IntakeLinearConstants.RIGHT_MOTOR_ID, IntakeLinearConstants.INVERTED);
        left = createSide(IntakeLinearConstants.LEFT_MOTOR_ID, !IntakeLinearConstants.INVERTED);
    }

    private MotorGroup createSide(int motorId, boolean isInverted) {
        MotorGroup side = addMotorGroup(new MotorGroup(Type.LINEAR, motorId, IntakeLinearConstants.VELOCITY,
                IntakeLinearConstants.ACCELERATION, IntakeLinearConstants.JERK,
                IntakeLinearConstants.LOWER_TOLERANCE,
                IntakeLinearConstants.UPPER_TOLERANCE,
                IntakeLinearConstants.GEAR_RATIOA, true, IntakeLinearConstants.CAN_BUS));

        side.configureMotor(isInverted, IntakeLinearConstants.BRAKE,
                IntakeLinearConstants.SUPPLY_CURRENT_LIMIT,
                IntakeLinearConstants.STATOR_CURRENT_LIMIT,
                IntakeLinearConstants.SLOT0_CONFIGS, false);

        side.configureLinearMech(2 * Math.PI, IntakeLinearConstants.MIN, IntakeLinearConstants.MAX);

        return side;
    }

    public static IntakeLinearSubsystem getInstance() {
        if (intakeLinearSubsystem == null) {
            intakeLinearSubsystem = new IntakeLinearSubsystem();
        }
        return intakeLinearSubsystem;
    }

    public MotorGroup getLeft() {
        return left;
    }

    public MotorGroup getRight() {
        return right;
    }

    @Override
    public void periodic() {
        super.periodic();
    }
}
