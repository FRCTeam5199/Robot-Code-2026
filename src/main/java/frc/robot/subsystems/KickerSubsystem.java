package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.KickerConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.templates.TemplateSubsystem;
import frc.robot.utility.Type;

public class KickerSubsystem extends TemplateSubsystem {
    private static KickerSubsystem kickerSubsystem;
    private VelocityDutyCycle bangBangController;
    private VelocityTorqueCurrentFOC torqueCurrentFOC;
    private boolean isUsingBangBangControl = true;
    private double goalVelocity = 0;

    private KickerSubsystem() {
        super(Type.ROLLER, KickerConstants.KICKER_LOWER_MOTOR_ID,
                0, KickerConstants.KICKER_ACCELERATION,
                KickerConstants.KICKER_JERK,
                KickerConstants.KICKER_LOWER_TOLERANCE,
                KickerConstants.KICKER_UPPER_TOLERANCE,
                KickerConstants.KICKER_GEAR_RATIO, "Kicker");

        configureMotor(KickerConstants.KICKER_LOWER_INVERTED, KickerConstants.KICKER_BRAKE,
                KickerConstants.KICKER_SUPPLY_CURRENT_LIMIT,
                KickerConstants.KICKER_STATOR_CURRENT_LIMIT,
                KickerConstants.KICKER_SLOT0_CONFIGS);

        configureFollowerMotor(
                KickerConstants.KICKER_UPPER_MOTOR_ID,
                KickerConstants.KICKER_UPPER_INVERTED
        );

//        getMotor().getConfigurator().apply(KickerConstants.KICKER_SLOT0_CONFIGS);
//        getMotor().getConfigurator().apply(KickerConstants.KICKER_SLOT1_CONFIGS);
//        getMotor().getConfigurator().apply(
//                new TorqueCurrentConfigs().withPeakForwardTorqueCurrent(60));

//        bangBangController = new VelocityDutyCycle(0)
//                .withSlot(0).withEnableFOC(true);
//        torqueCurrentFOC = new VelocityTorqueCurrentFOC(0).withSlot(1);
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
//        System.out.println("Kicker Velocity: " + getMotorVelocity());
        // System.out.println("Goal: " + goalVelocity);
//        System.out.println("Kicker is at goal: " + isMechAtGoal());

//        if (goalVelocity == 0) kickerSubsystem.setPercent(0);
//        else {
//            if (isMechAtGoal()) {
//                setVelocity(goalVelocity);
//            } else {
//                setVelocityBangBang(goalVelocity);
//            }
//        }
    }

//    public void setVelocityBangBang(double velocity) {
//        goalVelocity = velocity;
//        getMotor().setControl(bangBangController.withVelocity(velocity));
//    }
//
//    public void setSpeedTorqueCurrent(double velocity) {
//        goalVelocity = velocity;
//        getMotor().setControl(torqueCurrentFOC.withVelocity(velocity));
//    }
//
//    public boolean isMechAtGoal() {
//        return getMotorVelocity() >= goalVelocity - KickerConstants.KICKER_LOWER_TOLERANCE
//                && getMotorVelocity() <= goalVelocity + KickerConstants.KICKER_UPPER_TOLERANCE;
//    }
}
