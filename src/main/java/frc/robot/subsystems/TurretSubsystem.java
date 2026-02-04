package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.TurretConstants;
import frc.robot.subsystems.templates.TemplateSubsystem;
import frc.robot.utility.Type;

public class TurretSubsystem extends TemplateSubsystem {
    private static TurretSubsystem turretSubsystem;

    private double encoderRotationsContinuous;
    private double lastEncoderRotation;

    private TurretSubsystem() {
        super(Type.ROLLER, TurretConstants.TURRET_MOTOR_ID,
                TurretConstants.TURRET_VELOCITY, TurretConstants.TURRET_ACCELERATION,
                TurretConstants.TURRET_JERK,
                TurretConstants.TURRET_LOWER_TOLERANCE,
                TurretConstants.TURRET_UPPER_TOLERANCE,
                TurretConstants.TURRET_GEAR_RATIO, "ROBOT_TO_TURRET");

        configureMotor(TurretConstants.TURRET_INVERTED, TurretConstants.TURRET_BRAKE,
                TurretConstants.TURRET_SUPPLY_CURRENT_LIMIT,
                TurretConstants.TURRET_STATOR_CURRENT_LIMIT,
                TurretConstants.TURRET_SLOT0_CONFIGS);

        halfConfigureEncoder(TurretConstants.TURRET_ENCODER_ID,
                "rio", TurretConstants.TURRET_ENCODER_MAGNET_OFFSET,
                TurretConstants.TURRET_SENSOR_TO_MECH_GEAR_RATIO,
                TurretConstants.TURRET_MOTOR_TO_SENSOR_GEAR_RATIO,
                TurretConstants.TURRET_CCW_POSITIVE, TurretConstants.TURRET_ABSOLUTE_DISCONTINUITY_POINT);

        configureRoller(TurretConstants.TURRET_MIN, TurretConstants.TURRET_MAX);

//        encoderRotationsContinuous = getEncoderRot();
//        lastEncoderRotation = getEncoderRot();
    }

    public static TurretSubsystem getInstance() {
        if (turretSubsystem == null) {
            turretSubsystem = new TurretSubsystem();
        }
        return turretSubsystem;
    }

    @Override
    public void periodic() {
        super.periodic();
//        if (getEncoder() != null) {
//            double currentEncoderRotation = getEncoderRot();
//            double deltaEncoderRotations = 0;
//            if (currentEncoderRotation < .1 && lastEncoderRotation > .9) {
//                deltaEncoderRotations = currentEncoderRotation + (1 - lastEncoderRotation);
//            } else if (currentEncoderRotation > .9 && lastEncoderRotation < .1) {
//                deltaEncoderRotations = (currentEncoderRotation - 1) - lastEncoderRotation;
//            } else {
//                deltaEncoderRotations = currentEncoderRotation - lastEncoderRotation;
//            }
//            encoderRotationsContinuous += deltaEncoderRotations;
//
//            lastEncoderRotation = currentEncoderRotation;
//        }
//        System.out.println(encoderRotationsContinuous * TurretConstants.TURRET_SENSOR_TO_MECH_GEAR_RATIO * 360d);
        System.out.println("Degrees " + getDegrees());
//        System.out.println("Turret is at goal: " + isMechAtGoal(false));
    }
}
