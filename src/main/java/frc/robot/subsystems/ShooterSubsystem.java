package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
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
                ShooterConstants.SHOOTER_GEAR_RATIO, "Shooter");

        configureMotor(ShooterConstants.SHOOTER_INVERTED, ShooterConstants.SHOOTER_BRAKE,
                ShooterConstants.SHOOTER_SUPPLY_CURRENT_LIMIT,
                ShooterConstants.SHOOTER_STATOR_CURRENT_LIMIT,
                ShooterConstants.SHOOTER_SLOT0_CONFIGS);

        configureFollowerMotor(ShooterConstants.SECOND_SHOOTER_MOTOR_ID,
                ShooterConstants.SECOND_SHOOTER_INVERTED
        );

//        getMotor().getConfigurator().apply(ShooterConstants.SHOOTER_SLOT1_CONFIGS);
//        getMotor().getConfigurator().apply(
//                new TorqueCurrentConfigs().withPeakForwardTorqueCurrent(60));

//        bangBangController = new VelocityDutyCycle(0)
//                .withSlot(0).withEnableFOC(true);
//        torqueCurrentFOC = new VelocityTorqueCurrentFOC(0).withSlot(1);
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
    }
}
