package frc.robot.subsystems;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.templates.TemplateSubsystem;
import frc.robot.utility.ShotCalculator;
import frc.robot.utility.Type;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.units.Units.RotationsPerSecond;

public class ShooterSubsystem extends TemplateSubsystem {
    private static ShooterSubsystem shooterSubsystem;
    private static BooleanPublisher isMechAtGoal;
    private static DoublePublisher goalSpeed;
    private static DoublePublisher currentSpeed;
    private static NetworkTable shooterNetworkTable;
    private static ShotCalculator shotCalculator = ShotCalculator.getInstance();

//    private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
//            new SysIdRoutine.Config(
//                    Volts.of(0.2).per(Second), // ramp rate - slow for a turret
//                    Volts.of(6),                       // max voltage - keep low for turret safety
//                    Seconds.of(8),                     // test timeout
//                    null
//            ),
//            new SysIdRoutine.Mechanism(
//                    (voltage) -> setVoltage(voltage.in(Volts)),
//                    log -> {
//                        log.motor("shooter")
//                                .voltage(Volts.of(getMotor().getMotorVoltage().getValueAsDouble()))
//                                .angularPosition(Rotations.of(getMotor().getRotorPosition().getValueAsDouble()))
//                                .angularVelocity(RotationsPerSecond.of(getMotor().getRotorVelocity().getValueAsDouble()));
//                    },
//                    this
//            )
//    );

    private ShooterSubsystem() {
        super(Type.ROLLER, ShooterConstants.MOTOR_ID,
                0, ShooterConstants.ACCELERATION,
                ShooterConstants.JERK,
                ShooterConstants.LOWER_TOLERANCE,
                ShooterConstants.UPPER_TOLERANCE,
                ShooterConstants.GEAR_RATIO, "Shooter", true, ShooterConstants.CANBUS);

        configureMotor(ShooterConstants.INVERTED, ShooterConstants.BRAKE,
                ShooterConstants.SUPPLY_CURRENT_LIMIT,
                ShooterConstants.STATOR_CURRENT_LIMIT,
                ShooterConstants.SLOT0_CONFIGS);

        configureFollowerMotor(ShooterConstants.SECOND_MOTOR_ID,
                ShooterConstants.SECOND_INVERTED,
                ShooterConstants.CANBUS
        );

//        shooterNetworkTable ][tworkTable.getDoubleTopic("Current Speed").publish();
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
//        isMechAtGoal.set(isMechAtGoalAuto());
//        goalSpeed.set(shotCalculator.getShooterSpeed());
//        currentSpeed.set(getMotorVelocity());

//        System.out.println("shooter: " + isMechAtGoal(true));
    }

    public boolean isMechAtGoalAuto() {
        return getMotorVelocity() >= shotCalculator.getShooterSpeed() - ShooterConstants.LOWER_TOLERANCE
                && getMotorVelocity() <= shotCalculator.getShooterSpeed() + ShooterConstants.UPPER_TOLERANCE;
    }

//    public Command sysIdQuasistaticForward() {
//        return sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward);
//    }
//
//    public Command sysIdQuasistaticReverse() {
//        return sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse);
//    }
//
//    public Command sysIdDynamicForward() {
//        return sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward);
//    }
//
//    public Command sysIdDynamicReverse() {
//        return sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse);
//    }
}
