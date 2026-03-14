package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.RobotCentric;
import com.fasterxml.jackson.databind.EnumNamingStrategies.KebabCaseStrategy;

import frc.robot.RobotContainer;
import frc.robot.constants.KickerConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.templates.TemplateSubsystem;
import frc.robot.utility.ShotCalculator;
import frc.robot.utility.ShotMode;
import frc.robot.utility.Type;

public class KickerSubsystem extends TemplateSubsystem {
    private static KickerSubsystem kickerSubsystem;
    private static ShotCalculator shotCalculator = ShotCalculator.getInstance();

    private KickerSubsystem() {
        super(Type.ROLLER, KickerConstants.UPPER_MOTOR_ID,
                0, KickerConstants.ACCELERATION,
                KickerConstants.JERK,
                KickerConstants.LOWER_TOLERANCE,
                KickerConstants.UPPER_TOLERANCE,
                KickerConstants.GEAR_RATIO, "Kicker", true);

        configureMotor(KickerConstants.UPPER_INVERTED, KickerConstants.UPPER_BRAKE,
                KickerConstants.SUPPLY_CURRENT_LIMIT,
                KickerConstants.STATOR_CURRENT_LIMIT,
                KickerConstants.UPPER_SLOT0_CONFIGS);

        configureFollowerMotor(KickerConstants.LOWER_MOTOR_ID, KickerConstants.LOWER_INVERTED);
    }

    public static KickerSubsystem getInstance() {
        if (kickerSubsystem == null) {
            kickerSubsystem = new KickerSubsystem();
        }
        return kickerSubsystem;
    }

    @Override
    public void periodic() {
        super.periodic();
        // System.out.println("kicker: " + isMechAtGoal(true));
//        System.out.println(getGoal());
//        System.out.println("Vel: " + getMotorVelocity());

    }

    public boolean isMechAtGoalAuto() {
        if (RobotContainer.getShotMode() != ShotMode.SHOOTING) return true;
        return getMotorVelocity() >= shotCalculator.getKickerSpeed() - KickerConstants.LOWER_TOLERANCE
                && getMotorVelocity() <= shotCalculator.getKickerSpeed() + KickerConstants.UPPER_TOLERANCE;
    }
}
