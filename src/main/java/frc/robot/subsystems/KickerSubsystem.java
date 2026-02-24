package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.fasterxml.jackson.databind.EnumNamingStrategies.KebabCaseStrategy;

import frc.robot.constants.KickerConstants;
import frc.robot.subsystems.templates.TemplateSubsystem;
import frc.robot.utility.Type;

public class KickerSubsystem extends TemplateSubsystem {
    private static KickerSubsystem kickerSubsystem;

    private KickerSubsystem() {
        super(Type.ROLLER, KickerConstants.UPPER_MOTOR_ID,
                0, KickerConstants.ACCELERATION,
                KickerConstants.JERK,
                KickerConstants.LOWER_TOLERANCE,
                KickerConstants.UPPER_TOLERANCE,
                KickerConstants.GEAR_RATIO, "Kicker");

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
//        System.out.println(isMechAtGoal(true));

    }

    public double getKickerLowerSpeedFromUpperSpeed(double upperKickerSpeed) {
        return upperKickerSpeed * KickerConstants.SCALE_FACTOR;
    }
}
