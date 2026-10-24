package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.KickerConstants;
import frc.robot.subsystems.templates.TemplateSubsystem;
import frc.robot.utility.Type;

public class KickerSubsystem extends TemplateSubsystem {
    private static KickerSubsystem kickerSubsystem;

    private KickerSubsystem() {
        super(Type.ROLLER, KickerConstants.MOTOR_ID,
                0, KickerConstants.ACCELERATION,
                KickerConstants.JERK,
                KickerConstants.LOWER_TOLERANCE,
                KickerConstants.UPPER_TOLERANCE,
                KickerConstants.GEAR_RATIO, "Indexer", true, KickerConstants.canbus);

        configureMotor(KickerConstants.INVERTED, KickerConstants.BRAKE,
                KickerConstants.SUPPLY_CURRENT_LIMIT,
                KickerConstants.STATOR_CURRENT_LIMIT,
                KickerConstants.SLOT0_CONFIGS, false);

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
//        System.out.println("Indexer Velocity: " + isMechAtGoal(true));
//        System.out.println("Indexer Goal: " + getGoal());

    }
}
