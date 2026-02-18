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
        super(Type.ROLLER, KickerConstants.KICKER_UPPER_MOTOR_ID,
                0, KickerConstants.KICKER_ACCELERATION,
                KickerConstants.KICKER_JERK,
                KickerConstants.KICKER_LOWER_TOLERANCE,
                KickerConstants.KICKER_UPPER_TOLERANCE,
                KickerConstants.KICKER_GEAR_RATIO, "Kicker");

        configureMotor(KickerConstants.KICKER_UPPER_INVERTED, KickerConstants.KICKER_UPPER_BRAKE,
                KickerConstants.KICKER_SUPPLY_CURRENT_LIMIT,
                KickerConstants.KICKER_STATOR_CURRENT_LIMIT,
                KickerConstants.KICKER_UPPER_SLOT0_CONFIGS);

        // configureSecondaryMotor(KickerConstants.KICKER_LOWER_MOTOR_ID,
        // 0, KickerConstants.KICKER_ACCELERATION,
        //     KickerConstants.KICKER_JERK,
        //     KickerConstants.KICKER_LOWER_INVERTED,
        //     KickerConstants.KICKER_LOWER_BRAKE,
        //     KickerConstants.KICKER_SUPPLY_CURRENT_LIMIT,
        //     KickerConstants.KICKER_STATOR_CURRENT_LIMIT,
        //     KickerConstants.KICKER_LOWER_SLOT0_CONFIGS);

        configureFollowerMotor(KickerConstants.KICKER_LOWER_MOTOR_ID, KickerConstants.KICKER_LOWER_INVERTED);
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

//        goalVelocity = getGoal();
    //    System.out.println("Kicker Velocity: " + getMotorVelocity());
        // System.out.println("Upper Goal: " + getGoal());
        // System.out.println("Lower Goal: " + getSecondaryGoal());
    //    System.out.println("Kicker is at goal: " + isMechAtGoal(true));

//        if (goalVelocity == 0) kickerSubsystem.setPercent(0);
//        else {
//            if (isMechAtGoal()) {
//                setVelocity(goalVelocity);
//            } else {
//                setVelocityBangBang(goalVelocity);
//            }
//        }
    }

    public double getKickerLowerSpeedFromUpperSpeed(double upperKickerSpeed) {
        return upperKickerSpeed * KickerConstants.KICKER_SCALE_FACTOR;
    }
}
