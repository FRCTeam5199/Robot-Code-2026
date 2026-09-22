package frc.robot.subsystems;

import static org.wpilib.units.Units.Rotations;
import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.Second;
import static org.wpilib.units.Units.Seconds;
import static org.wpilib.units.Units.Volts;

import org.wpilib.command2.Command;
import org.wpilib.command2.sysid.SysIdRoutine;

import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.templates.TemplateSubsystem;
import frc.robot.utility.ShotCalculator;
import frc.robot.utility.SubsystemType;

public class ShooterSubsystem extends TemplateSubsystem {
    private static ShooterSubsystem shooterSubsystem;
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

    private ShooterSubsystem() {
        super(SubsystemType.ROLLER, ShooterConstants.MOTOR_ID, ShooterConstants.CANBUS,
                0, ShooterConstants.ACCELERATION,
                ShooterConstants.JERK,
                ShooterConstants.LOWER_TOLERANCE,
                ShooterConstants.UPPER_TOLERANCE,
                ShooterConstants.GEAR_RATIO, "Shooter", true);

        configureMotor(ShooterConstants.INVERTED, ShooterConstants.BRAKE,
                ShooterConstants.SUPPLY_CURRENT_LIMIT,
                ShooterConstants.STATOR_CURRENT_LIMIT,
                ShooterConstants.SLOT0_CONFIGS, true);

        configureFollowerMotor(ShooterConstants.SECOND_MOTOR_ID, 
                ShooterConstants.CANBUS, 
                ShooterConstants.SECOND_INVERTED
        );
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
    }

    public boolean isMechAtGoalAuto() {
        return getMotorVelocity() >= shotCalculator.getShooterSpeed() - ShooterConstants.LOWER_TOLERANCE
                && getMotorVelocity() <= shotCalculator.getShooterSpeed() + ShooterConstants.UPPER_TOLERANCE;
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
