package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotContainer;
import frc.robot.constants.IndexerConstants;
import frc.robot.subsystems.templates.TemplateSubsystem;
import frc.robot.utility.ShotCalculator;
import frc.robot.utility.ShotMode;
import frc.robot.utility.Type;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.units.Units.RotationsPerSecond;

public class IndexerSubsystem extends TemplateSubsystem {
    private static IndexerSubsystem indexerSubsystem;
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
                        log.motor("indexer")
                                .voltage(Volts.of(getMotor().getMotorVoltage().getValueAsDouble()))
                                .angularPosition(Rotations.of(getMotor().getRotorPosition().getValueAsDouble()))
                                .angularVelocity(RotationsPerSecond.of(getMotor().getRotorVelocity().getValueAsDouble()));
                    },
                    this
            )
    );

//    private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
//            new SysIdRoutine.Config(
//                    Volts.of(1).per(Second), // ramp rate - slow for a turret
//                    Volts.of(6),                       // max voltage - keep low for turret safety
//                    Seconds.of(8),                     // test timeout
//                    null
//            ),
//            new SysIdRoutine.Mechanism(
//                    (voltage) -> setVoltage(voltage.in(Volts)),
//                    log -> {
//                        log.motor("indexer")
//                                .voltage(Volts.of(getSecondaryMotor().getMotorVoltage().getValueAsDouble()))
//                                .angularPosition(Rotations.of(getSecondaryMotor().getRotorPosition().getValueAsDouble()))
//                                .angularVelocity(RotationsPerSecond.of(getSecondaryMotor().getRotorVelocity().getValueAsDouble()));
//                    },
//                    this
//            )
//    );

    private IndexerSubsystem() {
        super(Type.ROLLER, IndexerConstants.UPPER_MOTOR_ID,
                0, IndexerConstants.ACCELERATION,
                IndexerConstants.JERK,
                IndexerConstants.LOWER_TOLERANCE,
                IndexerConstants.UPPER_TOLERANCE,
                IndexerConstants.GEAR_RATIO, "Indexer", true, IndexerConstants.canbus);

        configureMotor(IndexerConstants.UPPER_INVERTED, IndexerConstants.UPPER_BRAKE,
                IndexerConstants.UPPER_SUPPLY_CURRENT_LIMIT,
                IndexerConstants.UPPER_STATOR_CURRENT_LIMIT,
                IndexerConstants.UPPER_SLOT0_CONFIGS, false);

        configureFollowerMotor(IndexerConstants.SECOND_UPPER_MOTOR_ID, IndexerConstants.SECONDARY_UPPER_INVERTED, IndexerConstants.canbus);

        configureSecondaryMotor(IndexerConstants.LOWER_MOTOR_ID, IndexerConstants.LOWER_INVERTED,
                IndexerConstants.LOWER_BRAKE,
                IndexerConstants.LOWER_SUPPLY_CURRENT_LIMIT,
                IndexerConstants.LOWER_STATOR_CURRENT_LIMIT,
                IndexerConstants.LOWER_SLOT0_CONFIGS);
    }

    public static IndexerSubsystem getInstance() {
        if (indexerSubsystem == null) {
            indexerSubsystem = new IndexerSubsystem();
        }
        return indexerSubsystem;
    }

    @Override
    public void periodic() {
        super.periodic();
//        System.out.println("Indexer Velocity: " + isMechAtGoal(true));
//        System.out.println("Indexer Goal: " + getGoal());

    }

    public boolean isMechAtGoalAuto() {
        if (RobotContainer.getShotMode() != ShotMode.SHOOTING) return true;
        return getMotorVelocity() >= IndexerConstants.UPPER_INDEXER_SPEED - IndexerConstants.LOWER_TOLERANCE
                && getMotorVelocity() <= IndexerConstants.UPPER_INDEXER_SPEED + IndexerConstants.UPPER_TOLERANCE;
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
