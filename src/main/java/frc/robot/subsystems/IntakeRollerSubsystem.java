package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.IntakeRollerConstants;
import frc.robot.subsystems.templates.MotorGroup;
import frc.robot.subsystems.templates.TemplateSubsystem;
import frc.robot.utility.Type;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.units.Units.RotationsPerSecond;

public class IntakeRollerSubsystem extends TemplateSubsystem {
    private static IntakeRollerSubsystem intakeRollerSubsystem;

    private IntakeRollerSubsystem() {
        super("Intake Roller");

        MotorGroup motors = addMotorGroup(new MotorGroup(Type.ROLLER, IntakeRollerConstants.MOTOR_ID,
                0, IntakeRollerConstants.ACCELERATION,
                IntakeRollerConstants.JERK,
                IntakeRollerConstants.LOWER_TOLERANCE,
                IntakeRollerConstants.UPPER_TOLERANCE,
                IntakeRollerConstants.GEAR_RATIO, false, IntakeRollerConstants.CAN_BUS));

        motors.configureMotor(IntakeRollerConstants.INVERTED, IntakeRollerConstants.BRAKE,
                IntakeRollerConstants.SUPPLY_CURRENT_LIMIT,
                IntakeRollerConstants.STATOR_CURRENT_LIMIT,
                IntakeRollerConstants.SLOT0_CONFIGS, false);

        motors.addFollower(IntakeRollerConstants.FOLLOWER_MOTOR_ID, IntakeRollerConstants.FOLLOWER_INVERTED);
    }

    public static IntakeRollerSubsystem getInstance() {
        if (intakeRollerSubsystem == null) {
            intakeRollerSubsystem = new IntakeRollerSubsystem();
        }
        return intakeRollerSubsystem;
    }

    private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                    Volts.of(1).per(Second), // ramp rate - slow for a turret
                    Volts.of(6),                       // max voltage - keep low for turret safety
                    Seconds.of(8),                     // test timeout
                    null
            ),
            new SysIdRoutine.Mechanism(
                    (voltage) -> getMotorGroup().setVoltage(voltage.in(Volts)),
                    log -> {
                        log.motor("intakeRoller")
                                .voltage(Volts.of(getMotorGroup().getMotor().getMotorVoltage().getValueAsDouble()))
                                .angularPosition(Rotations.of(getMotorGroup().getMotor().getRotorPosition().getValueAsDouble()))
                                .angularVelocity(RotationsPerSecond.of(getMotorGroup().getMotor().getRotorVelocity().getValueAsDouble()));
                    },
                    this
            )
    );

    @Override
    public void periodic() {
        super.periodic();
    }

    public Command sysIdQuasistaticForward() {
        return sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward);
    }

    public Command sysIdQuasistaticReverse() {
        return sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse);
    }

    public Command sysIdDynamicForward() {
        return sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward);
    }

    public Command sysIdDynamicReverse() {
        return sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse);
    }
}
