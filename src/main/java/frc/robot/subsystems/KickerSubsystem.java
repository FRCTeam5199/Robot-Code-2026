package frc.robot.subsystems;

import static org.wpilib.units.Units.Rotations;
import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.Second;
import static org.wpilib.units.Units.Seconds;
import static org.wpilib.units.Units.Volts;

import org.wpilib.command2.Command;
import org.wpilib.command2.sysid.SysIdRoutine;

import frc.robot.RobotContainer;
import frc.robot.constants.KickerConstants;
import frc.robot.subsystems.templates.TemplateSubsystem;
import frc.robot.utility.ShotCalculator;
import frc.robot.utility.ShotMode;
import frc.robot.utility.SubsystemType;

public class KickerSubsystem extends TemplateSubsystem {
    private static KickerSubsystem kickerSubsystem;
    private static ShotCalculator shotCalculator = ShotCalculator.getInstance();

    private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                    Volts.of(1).per(Second), // ramp rate - slow for a turret
                    Volts.of(6),                       // max voltage - keep low for turret safety
                    Seconds.of(8),                     // test timeout
                    null
            ),
            new SysIdRoutine.Mechanism(
                    (voltage) -> setVoltage(voltage.in(Volts)),
                    log -> {
                        log.motor("shooter")
                                .voltage(Volts.of(getMotor().getMotorVoltage().getValueAsDouble()))
                                .angularPosition(Rotations.of(getMotor().getRotorPosition().getValueAsDouble()))
                                .angularVelocity(RotationsPerSecond.of(getMotor().getRotorVelocity().getValueAsDouble()));
                    },
                    this
            )
    );

    private KickerSubsystem() {
        super(SubsystemType.ROLLER, KickerConstants.UPPER_MOTOR_ID, KickerConstants.CANBUS, 
                0, KickerConstants.ACCELERATION,
                KickerConstants.JERK,
                KickerConstants.LOWER_TOLERANCE,
                KickerConstants.UPPER_TOLERANCE,
                KickerConstants.GEAR_RATIO, "Kicker", true);

        configureMotor(KickerConstants.UPPER_INVERTED, KickerConstants.UPPER_BRAKE,
                KickerConstants.SUPPLY_CURRENT_LIMIT,
                KickerConstants.STATOR_CURRENT_LIMIT,
                KickerConstants.UPPER_SLOT0_CONFIGS, false);

        configureFollowerMotor(KickerConstants.LOWER_MOTOR_ID, KickerConstants.CANBUS, KickerConstants.LOWER_INVERTED);
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
    }

    public boolean isMechAtGoalAuto() {
        if (RobotContainer.getShotMode() != ShotMode.SHOOTING) return true;
        return getMotorVelocity() >= shotCalculator.getKickerSpeed() - KickerConstants.LOWER_TOLERANCE
                && getMotorVelocity() <= shotCalculator.getKickerSpeed() + KickerConstants.UPPER_TOLERANCE;
    }

    public Command sysIdQuasistaticForward() {
        return sysIdRoutine.quasistatic(SysIdRoutine.Direction.FORWARD);
    }

    public Command sysIdQuasistaticReverse() {
        return sysIdRoutine.quasistatic(SysIdRoutine.Direction.REVERSE);
    }

    public Command sysIdDynamicForward() {
        return sysIdRoutine.dynamic(SysIdRoutine.Direction.FORWARD);
    }

    public Command sysIdDynamicReverse() {
        return sysIdRoutine.dynamic(SysIdRoutine.Direction.REVERSE);
    }
}
