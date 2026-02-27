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
        super(Type.ROLLER, ShooterConstants.MOTOR_ID,
                0, ShooterConstants.ACCELERATION,
                ShooterConstants.JERK,
                ShooterConstants.LOWER_TOLERANCE,
                ShooterConstants.UPPER_TOLERANCE,
                ShooterConstants.GEAR_RATIO, "Shooter");

        configureMotor(ShooterConstants.INVERTED, ShooterConstants.BRAKE,
                ShooterConstants.SUPPLY_CURRENT_LIMIT,
                ShooterConstants.STATOR_CURRENT_LIMIT,
                ShooterConstants.SLOT0_CONFIGS);

        configureFollowerMotor(ShooterConstants.SECOND_MOTOR_ID,
                ShooterConstants.SECOND_INVERTED
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
        System.out.println("Shooter:" + isMechAtGoal(true));
    }
}
