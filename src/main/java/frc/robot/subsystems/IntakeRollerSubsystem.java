package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.IntakeRollerConstants;
import frc.robot.subsystems.templates.TemplateSubsystem;
import frc.robot.utility.Type;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.units.Units.RotationsPerSecond;

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
        super(Type.ROLLER, IntakeRollerConstants.MOTOR_ID,
                0, IntakeRollerConstants.ACCELERATION,
                IntakeRollerConstants.JERK,
                IntakeRollerConstants.LOWER_TOLERANCE,
                IntakeRollerConstants.UPPER_TOLERANCE,
                IntakeRollerConstants.GEAR_RATIO, "Intake Roller", false, IntakeRollerConstants.CAN_BUS);

        configureMotor(IntakeRollerConstants.INVERTED, IntakeRollerConstants.BRAKE,
                IntakeRollerConstants.SUPPLY_CURRENT_LIMIT,
                IntakeRollerConstants.STATOR_CURRENT_LIMIT,
                IntakeRollerConstants.SLOT0_CONFIGS, false);

        configureFollowerMotor(IntakeRollerConstants.FOLLOWER_MOTOR_ID,
                IntakeRollerConstants.FOLLOWER_INVERTED, IntakeRollerConstants.CAN_BUS, true);
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
        //    System.out.println("Velocity: " + getMotorVelocity());
        // System.out.println("Goal: " + getGoal());
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
