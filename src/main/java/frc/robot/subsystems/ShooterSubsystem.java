package frc.robot.subsystems;

import org.wpilib.networktables.BooleanPublisher;
import org.wpilib.networktables.DoublePublisher;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.command2.Command;
import org.wpilib.command2.sysid.SysIdRoutine;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.templates.TemplateSubsystem;
import frc.robot.utility.ShotCalculator;
import frc.robot.utility.Type;

import static org.wpilib.units.Units.*;
import static org.wpilib.units.Units.RotationsPerSecond;

public class ShooterSubsystem extends TemplateSubsystem {
    private static ShooterSubsystem shooterSubsystem;
    private static BooleanPublisher isMechAtGoal;
    private static DoublePublisher goalSpeed;
    private static DoublePublisher currentSpeed;
    private static NetworkTable shooterNetworkTable;
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
        super(Type.ROLLER, ShooterConstants.MOTOR_ID,
                0, ShooterConstants.ACCELERATION,
                ShooterConstants.JERK,
                ShooterConstants.LOWER_TOLERANCE,
                ShooterConstants.UPPER_TOLERANCE,
                ShooterConstants.GEAR_RATIO, "Shooter", true, ShooterConstants.CANBUS);

        configureMotor(ShooterConstants.INVERTED, ShooterConstants.BRAKE,
                ShooterConstants.SUPPLY_CURRENT_LIMIT,
                ShooterConstants.STATOR_CURRENT_LIMIT,
                ShooterConstants.SLOT0_CONFIGS, true);

        configureFollowerMotor(ShooterConstants.SECOND_MOTOR_ID,
                ShooterConstants.SECOND_INVERTED,
                ShooterConstants.CANBUS
        );
        
//        shooterNetworkTable.getDoubleTopic("Current Speed").publish();
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
