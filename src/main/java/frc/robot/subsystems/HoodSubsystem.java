package frc.robot.subsystems;

import frc.robot.constants.HoodConstants;
import frc.robot.subsystems.templates.TemplateSubsystem;
import frc.robot.utility.Type;

public class HoodSubsystem extends TemplateSubsystem {
    private static HoodSubsystem hoodSubsystem;

    private HoodSubsystem() {
        super(Type.PIVOT, HoodConstants.MOTOR_ID, HoodConstants.VELOCITY,
                HoodConstants.ACCELERATION, HoodConstants.JERK,
                HoodConstants.LOWER_TOLERANCE,
                HoodConstants.UPPER_TOLERANCE,
                HoodConstants.GEAR_RATIO, "Hood");

        configureMotor(HoodConstants.INVERTED, HoodConstants.BRAKE,
                HoodConstants.SUPPLY_CURRENT_LIMIT,
                HoodConstants.STATOR_CURRENT_LIMIT,
                HoodConstants.SLOT0_CONFIGS);

        configureSometimesEncoder(HoodConstants.ENCODER_ID,
                "rio", HoodConstants.MAGNET_OFFSET,
                HoodConstants.SENSOR_TO_MECH_GEAR_RATIO,
                HoodConstants.MOTOR_TO_SENSOR_GEAR_RATIO,
                HoodConstants.IS_CCW_POS,
                HoodConstants.ABSOLUTE_DISCONTINUITY_POINT);

        configurePivot(HoodConstants.MIN,
                HoodConstants.MAX);
    }

    public static HoodSubsystem getInstance() {
        if (hoodSubsystem == null) {
            hoodSubsystem = new HoodSubsystem();
        }
        return hoodSubsystem;
    }

    public void periodic() {
        super.periodic();
//        System.out.println("Hood Degrees: " + getDegrees());
//        System.out.println("Hood is at goal: " + isMechAtGoal(false));
//        System.out.println(getGearRatio());

        //Motor Rotations = degrees / 360 / .00694444444444
        //Degrees = motorRot * 360 * .00694444444444
    }
}

