package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.templates.TemplateSubsystem;
import frc.robot.utility.Type;

public class ShooterSubsystem extends TemplateSubsystem {
    private static ShooterSubsystem shooterSubsystem;
    private static BooleanPublisher isMechAtGoal;
    private static NetworkTable shooterNetworkTable;

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

        shooterNetworkTable = NetworkTableInstance.getDefault().getTable("Subsystems/Shooter/");
        isMechAtGoal = shooterNetworkTable.getBooleanTopic("Is Mech At Goal").publish();
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
        isMechAtGoal.set(isMechAtGoal(true));
    }
}
