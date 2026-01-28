package frc.robot.subsystems;

import frc.robot.constants.TurretConstants;
import frc.robot.subsystems.templates.TemplateSubsystem;
import frc.robot.utility.Type;

public class TurretSubsystem extends TemplateSubsystem {
    private static TurretSubsystem turretSubsystem;

    private TurretSubsystem() {
        super(Type.ROLLER, TurretConstants.TURRET_MOTOR_ID,
                0, TurretConstants.TURRET_ACCELERATION,
                TurretConstants.TURRET_JERK,
                TurretConstants.TURRET_LOWER_TOLERANCE,
                TurretConstants.TURRET_UPPER_TOLERANCE,
                TurretConstants.TURRET_GEAR_RATIO, "TURRET");

        configureMotor(TurretConstants.TURRET_INVERTED, TurretConstants.TURRET_BRAKE,
                TurretConstants.TURRET_SUPPLY_CURRENT_LIMIT,
                TurretConstants.TURRET_STATOR_CURRENT_LIMIT,
                TurretConstants.TURRET_SLOT0_CONFIGS);
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
//        System.out.println("Velocity: " + getMotorVelocity());
    }
}
