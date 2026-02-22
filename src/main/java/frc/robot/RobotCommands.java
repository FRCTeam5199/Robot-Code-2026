package frc.robot;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.constants.HopperConstants;
import frc.robot.constants.IndexerConstants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.templates.PositionCommand;
import frc.robot.subsystems.templates.TurretCommand;
import frc.robot.subsystems.templates.VelocityCommand;

public class RobotCommands {
    private static final KickerSubsystem kickerSubsystem = KickerSubsystem.getInstance();
    private static final IndexerSubsystem indexerSubsystem = IndexerSubsystem.getInstance();
    private static final HopperSubsystem hopperSubsystem = HopperSubsystem.getInstance();
    private static final TurretSubsystem turretSubsystem = TurretSubsystem.getInstance();
    private static final HoodSubsystem hoodSubsystem = HoodSubsystem.getInstance();
    private static final ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();
    private static boolean isIdling = false;

    public static Command indexBalls() {
        return new FunctionalCommand(
                () -> {
                    if (RobotContainer.areMechanismsAtGoals() && isIdling) {
                        isIdling = false;
                        indexerSubsystem.setVelocity(IndexerConstants.INDEXING_SPEED);
                        hopperSubsystem.setVelocity(HopperConstants.INDEXING_SPEED);
                    } else if (!RobotContainer.areMechanismsAtGoals() && !isIdling) {
                        isIdling = true;
                        indexerSubsystem.setVelocity(IndexerConstants.IDLING_SPEED);
                        hopperSubsystem.setVelocity(HopperConstants.IDLING_SPEED);
                    }
                },
                () -> {
                    if (RobotContainer.areMechanismsAtGoals() && isIdling) {
                        isIdling = false;
                        indexerSubsystem.setVelocity(IndexerConstants.INDEXING_SPEED);
                        hopperSubsystem.setVelocity(HopperConstants.INDEXING_SPEED);
                    } else if (!RobotContainer.areMechanismsAtGoals() && !isIdling) {
                        isIdling = true;
                        indexerSubsystem.setVelocity(IndexerConstants.IDLING_SPEED);
                        hopperSubsystem.setVelocity(HopperConstants.IDLING_SPEED);
                    }
                },
                (interrupted) -> {
                    indexerSubsystem.setVelocity(IndexerConstants.IDLING_SPEED);
                    hopperSubsystem.setVelocity(HopperConstants.IDLING_SPEED);
                },
                () -> false,
                indexerSubsystem, hopperSubsystem
        );
    }

    public static Command idleIndexerHopper() {
        return new ParallelCommandGroup(
                new VelocityCommand(indexerSubsystem, IndexerConstants.IDLING_SPEED),
                new VelocityCommand(hopperSubsystem, HopperConstants.IDLING_SPEED)
        );
    }

    public static Command zeroTurret() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> turretSubsystem.setStopMoving(true)),
                new PositionCommand(turretSubsystem, 0),
                new FunctionalCommand(
                        () -> {
                            if (turretSubsystem.getSometimesEncoderRot() < 0)
                                turretSubsystem.setVoltage(.5);
                            else
                                turretSubsystem.setVoltage(-.5);
                        },
                        () -> {
                        },
                        (interrupted) -> turretSubsystem.setVoltage(0),
                        () -> Math.abs(turretSubsystem.getSometimesEncoderRot()) < .1,
                        turretSubsystem
                ),
                new InstantCommand(turretSubsystem::zeroMotor),
                new InstantCommand(() -> turretSubsystem.setStopMoving(false)),
                new InstantCommand(() -> turretSubsystem.setPositionProfiling(0, 0))
        );
    }

    public static Command moveTurret(double degrees) {
        return new SequentialCommandGroup(
//                new InstantCommand(() -> turretSubsystem.setStopMoving(true)),
                new TurretCommand(turretSubsystem, degrees, 0)
        );
    }
}
