package frc.robot;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.constants.ClimberConstants;
import frc.robot.constants.HopperConstants;
import frc.robot.constants.IndexerConstants;
import frc.robot.constants.IntakePivotConstants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.templates.PositionCommand;
import frc.robot.subsystems.templates.VelocityCommand;
import frc.robot.utility.ClimberSetpoint;
import frc.robot.utility.ShotCalculator;

public class RobotCommands {
    public static final ClimberSubsystem climberSubsystem = ClimberSubsystem.getInstance();
    public static final IntakePivotSubsystem intakePivotSubsystem = IntakePivotSubsystem.getInstance();
    public static final ShotCalculator shotCalculator = ShotCalculator.getInstance();
    public static final IntakeRollerSubsystem intakeRollerSubsystem = IntakeRollerSubsystem.getInstance();
    private static final KickerSubsystem kickerSubsystem = KickerSubsystem.getInstance();
    private static final IndexerSubsystem indexerSubsystem = IndexerSubsystem.getInstance();
    private static final HopperSubsystem hopperSubsystem = HopperSubsystem.getInstance();
    private static final TurretSubsystem turretSubsystem = TurretSubsystem.getInstance();
    private static final HoodSubsystem hoodSubsystem = HoodSubsystem.getInstance();
    private static final ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();
    private static boolean isIdling = false;
    private static boolean isIndexing = false;

    // Runs Hopper and Indexer the way for shooting
    public static Command indexBallsAuto() {
        return new FunctionalCommand(
                () -> {
                    if (RobotContainer.areMechanismsAtGoalsAuto() && isIdling) {
                        isIdling = false;
                        hopperSubsystem.setVelocity(HopperConstants.INDEXING_SPEED);
                    } else if (!RobotContainer.areMechanismsAtGoalsAuto() && !isIdling) {
                        isIdling = true;
                        isIndexing = false;
                        indexerSubsystem.setVelocity(IndexerConstants.INDEXING_SPEED); //indexing speed
                        hopperSubsystem.setVelocity(HopperConstants.IDLING_SPEED);
                    }
                },
                () -> {
                    if (RobotContainer.areMechanismsAtGoalsAuto() && isIdling) {
                        isIdling = false;
                        hopperSubsystem.setVelocity(HopperConstants.INDEXING_SPEED);
                    } else if (!RobotContainer.areMechanismsAtGoalsAuto() && !isIdling) {
                        isIdling = true;
                        isIndexing = false;
                        indexerSubsystem.setVelocity(IndexerConstants.IDLING_SPEED);
                        hopperSubsystem.setVelocity(HopperConstants.IDLING_SPEED);
                    }

                    if (hopperSubsystem.isAboveSpeed(HopperConstants.INDEXING_SPEED - 10) && !isIndexing) {
                        isIndexing = true;
                        indexerSubsystem.setVelocity(IndexerConstants.INDEXING_SPEED); //indexing speed
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

    public static Command moveHoodToZero() {
        return new SequentialCommandGroup(
//                new InstantCommand(() -> hoodSubsystem.setStopMoving(true)),
//                new PositionCommand(hoodSubsystem, 0),
//                new InstantCommand(() -> hoodSubsystem.setStopMoving(false)),
                new InstantCommand(() -> hoodSubsystem.setPositionProfiling(0, 0))
        );
    }

    public static Command extendClimb() {
        return new InstantCommand(() ->
                climberSubsystem.setPosition(10));
    }


    public static Command retractClimb() {
        return new InstantCommand(() ->
                climberSubsystem.setPosition(0));
    }

    public static Command stopClimb() {
        return new InstantCommand(() ->
                climberSubsystem.setVelocity(0));
    }

    public static Command idleState() {
        return new ParallelCommandGroup(
                zeroTurret(),
                moveHoodToZero(),
                new VelocityCommand(indexerSubsystem, IndexerConstants.IDLING_SPEED),
                new VelocityCommand(hopperSubsystem, HopperConstants.IDLING_SPEED),
                new VelocityCommand(shooterSubsystem, 0),
                new VelocityCommand(kickerSubsystem, 0)
        );
    }

    public static Command teleopAutoClimbLeft() {
        return new SequentialCommandGroup(
                Autos.driveToPose(ClimberSetpoint.CLIMB_LEFT_RED_PREP)
                        .alongWith(new PositionCommand(climberSubsystem, ClimberConstants.DEPLOY))
                        .alongWith(new InstantCommand(() -> RobotContainer.setIsClimbing(true))),
                Autos.pidAlign(ClimberSetpoint.CLIMB_LEFT_RED),
                new PositionCommand(climberSubsystem, ClimberConstants.CLIMB)
        );
    }

    public static Command teleopAutoClimbRight() {
        return new SequentialCommandGroup(
                Autos.driveToPose(ClimberSetpoint.CLIMB_RIGHT_RED_PREP)
                        .alongWith(new PositionCommand(climberSubsystem, ClimberConstants.DEPLOY))
                        .alongWith(new InstantCommand(() -> RobotContainer.setIsClimbing(true))),
                Autos.pidAlign(ClimberSetpoint.CLIMB_RIGHT_RED),
                new PositionCommand(climberSubsystem, ClimberConstants.CLIMB)
        );
    }

    public static Command prepAutoCLimbLeft() {
        return new SequentialCommandGroup(
                Autos.driveToPose(ClimberSetpoint.CLIMB_LEFT_RED_PREP)
                        .alongWith(new PositionCommand(climberSubsystem, ClimberConstants.DEPLOY))
                        .alongWith(new InstantCommand(() -> RobotContainer.setIsClimbing(true))),
                Autos.pidAlign(ClimberSetpoint.CLIMB_LEFT_RED),
                new PositionCommand(intakePivotSubsystem, IntakePivotConstants.STOW)
        );
    }

    public static Command prepAutoCLimbRight() {
        return new SequentialCommandGroup(
                Autos.driveToPose(ClimberSetpoint.CLIMB_RIGHT_RED_PREP)
                        .alongWith(new PositionCommand(climberSubsystem, ClimberConstants.DEPLOY))
                        .alongWith(new InstantCommand(() -> RobotContainer.setIsClimbing(true))),
                Autos.pidAlign(ClimberSetpoint.CLIMB_RIGHT_RED),
                new PositionCommand(intakePivotSubsystem, IntakePivotConstants.STOW)
        );
    }

    public static Command outtake() {
        return new ParallelCommandGroup(
                new VelocityCommand(indexerSubsystem, -50),
                new VelocityCommand(hopperSubsystem, -50),
                new VelocityCommand(shooterSubsystem, -50),
                new VelocityCommand(kickerSubsystem, -50),
                new VelocityCommand(intakeRollerSubsystem, -50)
        );
    }
}
