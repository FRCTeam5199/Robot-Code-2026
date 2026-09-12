package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.constants.HopperConstants;
import frc.robot.constants.IndexerConstants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.templates.PositionCommand;
import frc.robot.subsystems.templates.VelocityCommand;
import frc.robot.utility.ShotCalculator;

public class RobotCommands {
    //    public static final ClimberSubsystem climberSubsystem = ClimberSubsystem.getInstance();
    public static final IntakePivotSubsystem intakePivotSubsystem = IntakePivotSubsystem.getInstance();
    public static final ShotCalculator shotCalculator = ShotCalculator.getInstance();
    public static final IntakeRollerSubsystem intakeRollerSubsystem = IntakeRollerSubsystem.getInstance();
    private static final IndexerSubsystem indexerSubsystem = IndexerSubsystem.getInstance();
    private static final HopperSubsystem hopperSubsystem = HopperSubsystem.getInstance();
    private static final TurretSubsystem turretSubsystem = TurretSubsystem.getInstance();
    private static final HoodSubsystem hoodSubsystem = HoodSubsystem.getInstance();
    private static final ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();
    private static boolean hasStartedTimer;
    private static Timer timer = new Timer();

    public static Command indexBallsAuto() {
        return new FunctionalCommand(
                () -> {
                    if (RobotContainer.areMechanismsAtGoalsAuto()) {
                        indexerSubsystem.setVelocity(IndexerConstants.UPPER_INDEXER_SPEED);
                        indexerSubsystem.setSecondaryVelocity(IndexerConstants.LOWER_INDEXER_SPEED);
                    }
                    if (indexerSubsystem.isMechAtGoal(true) && indexerSubsystem.getGoal() != 0) {
                        hopperSubsystem.setVelocity(HopperConstants.INDEXING_SPEED);
                    } else {
                        hopperSubsystem.setVelocity(-10);
                    }
                },
                () -> {
                    if (RobotContainer.areMechanismsExceptShooterAtGoalsAuto()) {
                        indexerSubsystem.setVelocity(IndexerConstants.UPPER_INDEXER_SPEED);
                        indexerSubsystem.setSecondaryVelocity(IndexerConstants.LOWER_INDEXER_SPEED);
                    } else {
                        indexerSubsystem.setVelocity(0);
                        indexerSubsystem.setSecondaryVelocity(0);
                    }

                    if (indexerSubsystem.isMechAtGoal(true) && indexerSubsystem.getGoal() != 0) {
                        hopperSubsystem.setVelocity(HopperConstants.INDEXING_SPEED);
                    } else {
                        hopperSubsystem.setVelocity(-10);
                    }
                },
                (interrupted) -> {
                    indexerSubsystem.setVelocity(0);
                    indexerSubsystem.setSecondaryVelocity(0);
                    hopperSubsystem.setVelocity(0);
                },
                () -> false,
                hopperSubsystem, indexerSubsystem
        );
    }

    public static Command indexBalls() {
        return new FunctionalCommand(
                () -> {
                    if (RobotContainer.areMechanismsAtGoalsAuto()) {
                        indexerSubsystem.setVelocity(IndexerConstants.UPPER_INDEXER_SPEED);
                        indexerSubsystem.setSecondaryVelocity(IndexerConstants.LOWER_INDEXER_SPEED);
                    }
                    if (indexerSubsystem.isMechAtGoal(true) && indexerSubsystem.getGoal() != 0) {
                        hopperSubsystem.setVelocity(HopperConstants.INDEXING_SPEED);
                    }
                },
                () -> {
                    if (RobotContainer.areMechanismsExceptShooterAtGoals()) {
                        indexerSubsystem.setVelocity(IndexerConstants.UPPER_INDEXER_SPEED);
                        indexerSubsystem.setSecondaryVelocity(IndexerConstants.LOWER_INDEXER_SPEED);
                    } else {
                        indexerSubsystem.setVelocity(0);
                        indexerSubsystem.setSecondaryVelocity(0);
                    }

                    if (indexerSubsystem.isMechAtGoal(true) && indexerSubsystem.getGoal() != 0) {
                        hopperSubsystem.setVelocity(HopperConstants.INDEXING_SPEED);
                    } else {
                        hopperSubsystem.setVelocity(0);
                    }

                },
                (interrupted) -> {
                    indexerSubsystem.setVelocity(0);
                    indexerSubsystem.setSecondaryVelocity(0);
                    hopperSubsystem.setVelocity(0);
                },
                () -> false,
                hopperSubsystem, indexerSubsystem
        );
    }

    public static Command zeroTurret() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> turretSubsystem.setStopMoving(true)),
                new PositionCommand(turretSubsystem, 0),
                new InstantCommand(turretSubsystem::zeroMotor),
//                new InstantCommand(() -> turretSubsystem.setStopMoving(false)),
                new InstantCommand(() -> turretSubsystem.setPositionProfiling(0, 0))
        );
    }

    public static Command moveHoodToZero() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> hoodSubsystem.setStopMoving(true)),
                new PositionCommand(hoodSubsystem, 0),
//                new InstantCommand(() -> hoodSubsystem.setStopMoving(false)),
                new InstantCommand(() -> hoodSubsystem.setPositionProfiling(0, 0))
        );
    }

    public static Command idleState() {
        return new ParallelCommandGroup(
                zeroTurret().onlyIf(() -> !turretSubsystem.fullStop),
                moveHoodToZero(),
                new VelocityCommand(hopperSubsystem, 0),
                new VelocityCommand(shooterSubsystem, 0),
                new VelocityCommand(indexerSubsystem, 0, 0)
        );
    }

    public static Command outtake() {
        return new ParallelCommandGroup(
                new VelocityCommand(hopperSubsystem, -50),
                new VelocityCommand(shooterSubsystem, -50),
                new VelocityCommand(indexerSubsystem, -50),
                new VelocityCommand(intakeRollerSubsystem, -50)
        );
    }
}
