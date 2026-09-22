package frc.robot.subsystems;

import static org.wpilib.units.Units.Rotations;
import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.Second;
import static org.wpilib.units.Units.Seconds;
import static org.wpilib.units.Units.Volts;

import org.wpilib.command2.Command;
import org.wpilib.command2.sysid.SysIdRoutine;

import frc.robot.subsystems.templates.TemplateSubsystem;
import frc.robot.utility.SubsystemType;
import frc.robot.constants.IntakeRollerConstants;

public class IntakeRollerSubsystem extends TemplateSubsystem {
    private static IntakeRollerSubsystem intakeRollerSubsystem;

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
                        log.motor("intake-roller")
                                .voltage(Volts.of(getMotor().getMotorVoltage().getValueAsDouble()))
                                .angularPosition(Rotations.of(getMotor().getRotorPosition().getValueAsDouble()))
                                .angularVelocity(RotationsPerSecond.of(getMotor().getRotorVelocity().getValueAsDouble()));
                    },
                    this
            )
    );

    private IntakeRollerSubsystem() {
        super(SubsystemType.ROLLER, IntakeRollerConstants.MOTOR_ID, IntakeRollerConstants.CAN_BUS, 
                0, IntakeRollerConstants.ACCELERATION,
                IntakeRollerConstants.JERK,
                IntakeRollerConstants.LOWER_TOLERANCE,
                IntakeRollerConstants.UPPER_TOLERANCE,
                IntakeRollerConstants.GEAR_RATIO, "Intake Roller", false);

        configureMotor(IntakeRollerConstants.INVERTED, IntakeRollerConstants.BRAKE,
                IntakeRollerConstants.SUPPLY_CURRENT_LIMIT,
                IntakeRollerConstants.STATOR_CURRENT_LIMIT,
                IntakeRollerConstants.SLOT0_CONFIGS, false);

        configureFollowerMotor(IntakeRollerConstants.FOLLOWER_MOTOR_ID, IntakeRollerConstants.CAN_BUS, 
                IntakeRollerConstants.FOLLOWER_INVERTED);
    }

    public static IntakeRollerSubsystem getInstance() {
        if (intakeRollerSubsystem == null) {
            intakeRollerSubsystem = new IntakeRollerSubsystem();
        }
        return intakeRollerSubsystem;
    }

    @Override
    public void periodic() {
        super.periodic();
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
