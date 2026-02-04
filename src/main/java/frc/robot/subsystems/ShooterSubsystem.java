package frc.robot.subsystems;

import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.templates.TemplateSubsystem;
import frc.robot.utility.Type;

public class ShooterSubsystem extends TemplateSubsystem {
    private static ShooterSubsystem shooterSubsystem;

    private ShooterSubsystem() {
        super(Type.ROLLER, ShooterConstants.SHOOTER_MOTOR_ID,
                0, ShooterConstants.SHOOTER_ACCELERATION,
                ShooterConstants.SHOOTER_JERK,
                ShooterConstants.SHOOTER_LOWER_TOLERANCE,
                ShooterConstants.SHOOTER_UPPER_TOLERANCE,
                ShooterConstants.SHOOTER_GEAR_RATIO, "SHOOTER");

        configureMotor(ShooterConstants.SHOOTER_INVERTED, ShooterConstants.SHOOTER_BRAKE,
                ShooterConstants.SHOOTER_SUPPLY_CURRENT_LIMIT,
                ShooterConstants.SHOOTER_STATOR_CURRENT_LIMIT,
                ShooterConstants.SHOOTER_SLOT0_CONFIGS);
    }

    public static ShooterSubsystem getInstance() {
        if (shooterSubsystem == null) {
            shooterSubsystem = new ShooterSubsystem();
        }
        return shooterSubsystem;
    }

    @Override
    public void periodic() {
        super.periodic();
        // System.out.println("Shooter Velocity: " + getMotorVelocity());
//      System.out.println("Shooter is at goal: " + isMechAtGoal(true));
    }
}
