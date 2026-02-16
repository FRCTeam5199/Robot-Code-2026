package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.templates.TemplateSubsystem;
import frc.robot.utility.Type;

public class ShooterSubsystem extends TemplateSubsystem {
    private static ShooterSubsystem shooterSubsystem;
    private VelocityDutyCycle bangBangController;
    private VelocityTorqueCurrentFOC torqueCurrentFOC;
    private boolean isUsingBangBangControl = true;
    private double goalVelocity = 0;

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
//        System.out.println("Shooter Velocity: " + getMotorVelocity());
//        System.out.println("Current Control Request: " + getMotor().getControlMode(true).getValue().name());
//     System.out.println("Shooter is at goal: " + isMechAtGoal(true));
//        if (goalVelocity == 0) shooterSubsystem.setPercent(0);
//        else {
//            if (isMechAtGoal()) {
//                setSpeedTorqueCurrent(goalVelocity);
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
//        return getMotorVelocity() >= goalVelocity - ShooterConstants.SHOOTER_LOWER_TOLERANCE
//                && getMotorVelocity() <= goalVelocity + ShooterConstants.SHOOTER_UPPER_TOLERANCE;
//    }
}
