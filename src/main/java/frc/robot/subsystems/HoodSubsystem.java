package frc.robot.subsystems;

import frc.robot.constants.HoodConstants;
import frc.robot.subsystems.templates.TemplateSubsystem;
import frc.robot.utility.Type;

public class HoodSubsystem extends TemplateSubsystem {
    private static HoodSubsystem hoodSubsystem;
    
    private HoodSubsystem() {
        super(Type.PIVOT, HoodConstants.HOOD_MOTOR_ID, HoodConstants.HOOD_VELOCITY,
                HoodConstants.HOOD_ACCELERATION, HoodConstants.HOOD_JERK,
                HoodConstants.HOOD_LOWER_TOLERANCE,
                HoodConstants.HOOD_UPPER_TOLERANCE,
                HoodConstants.HOOD_GEAR_RATIO, "HOOD");

        configureMotor(HoodConstants.HOOD_INVERTED, HoodConstants.HOOD_BRAKE,
                HoodConstants.HOOD_SUPPLY_CURRENT_LIMIT,
                HoodConstants.HOOD_STATOR_CURRENT_LIMIT,
                HoodConstants.HOOD_SLOT0_CONFIGS);

        configureEncoder(HoodConstants.HOOD_ENCODER_ID,
                "rio", HoodConstants.HOOD_MAGNET_OFFSET,
                HoodConstants.HOOD_SENSOR_TO_MECH_RATIO,
                HoodConstants.HOOD_MOTOR_TO_SENSOR_GEAR_RATIO,
                HoodConstants.HOOD_CCW_POSITIVE);

        configurePivot(HoodConstants.HOOD_MIN,
                HoodConstants.HOOD_MAX);
    }

    public static HoodSubsystem getInstance() {
        if (hoodSubsystem == null) {
            hoodSubsystem = new HoodSubsystem();
        }
        return hoodSubsystem;
    }

    public void periodic() {
        super.periodic();
//        System.out.println("Degrees: " + getDegrees());
//        System.out.println("Goal: " + getGoal());
    }
}
