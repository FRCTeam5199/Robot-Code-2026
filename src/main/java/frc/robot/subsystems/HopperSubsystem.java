package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.HopperConstants;
import frc.robot.subsystems.templates.TemplateSubsystem;
import frc.robot.utility.Type;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.units.Units.RotationsPerSecond;

public class HopperSubsystem extends TemplateSubsystem {
    private static HopperSubsystem hopperSubsystem;

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
                        log.motor("hopper")
                                .voltage(Volts.of(getMotor().getMotorVoltage().getValueAsDouble()))
                                .angularPosition(Rotations.of(getMotor().getRotorPosition().getValueAsDouble()))
                                .angularVelocity(RotationsPerSecond.of(getMotor().getRotorVelocity().getValueAsDouble()));
                    },
                    this
            )
    );

    private HopperSubsystem() {
        super(Type.ROLLER, HopperConstants.MOTOR_ID,
                0, HopperConstants.ACCELERATION,
                HopperConstants.JERK,
                HopperConstants.LOWER_TOLERANCE,
                HopperConstants.UPPER_TOLERANCE,
                HopperConstants.GEAR_RATIO, "Hopper", true, HopperConstants.canbus);

        configureMotor(HopperConstants.INVERTED, HopperConstants.BRAKE,
                HopperConstants.SUPPLY_CURRENT_LIMIT,
                HopperConstants.STATOR_CURRENT_LIMIT,
                HopperConstants.SLOT0_CONFIGS, false);
    }

    public static HopperSubsystem getInstance() {
        if (hopperSubsystem == null) {
            hopperSubsystem = new HopperSubsystem();
        }
        return hopperSubsystem;
    }

    @Override
    public void periodic() {
        super.periodic();
        //    System.out.println("Velocity: " + getMotorVelocity());
//        System.out.println(isMechAtGoal(true));
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