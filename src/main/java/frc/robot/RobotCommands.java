package frc.robot;

import frc.robot.constants.IntakePivotConstants;
import org.wpilib.command2.Command;
import org.wpilib.command2.FunctionalCommand;
import org.wpilib.command2.InstantCommand;
import org.wpilib.command2.ParallelCommandGroup;
import org.wpilib.command2.SequentialCommandGroup;
import org.wpilib.system.Timer;

import frc.robot.constants.HopperConstants;
import frc.robot.constants.IndexerConstants;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakePivotSubsystem;
import frc.robot.subsystems.IntakeRollerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
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
    private static double systemsOutOfToleranceCheck = 0;
    private static final double MAX_OUT_OF_TOLERANCE_TIMES = 10;
    private static final Timer intakeTimer = new Timer();
    private static boolean isIntakeAgitationDown = true;
    private static final double INTAKE_AGITATION_INTERVAL = .3;

    private static double hopperReverseCheck = 0;
    private static final double MAX_HOPPER_REVERSAL_TIMES = 3;
    private static double hopperStatorCurrentLimitCheck = 0;
    private static final double STATOR_CURRENT_TIMES = 1;

    public static Command indexBallsAuto() {
        return new FunctionalCommand(
                () -> {
//                    if (RobotContainer.areMechanismsAtGoalsAuto()) {
//                        indexerSubsystem.setVelocity(IndexerConstants.UPPER_INDEXER_SPEED);
//                        indexerSubsystem.setSecondaryVelocity(IndexerConstants.LOWER_INDEXER_SPEED);
//                    }
//                    if (indexerSubsystem.isMechAtGoal(true) && indexerSubsystem.getGoal() != 0) {
//                        hopperSubsystem.setVelocity(HopperConstants.INDEXING_SPEED);
//                    } else {
//                        hopperSubsystem.setVelocity(-10);
//                    }

                    if (RobotContainer.areMechanismsAtGoalsAuto()) {
                        indexerSubsystem.setVelocity(IndexerConstants.UPPER_INDEXER_SPEED);
                        indexerSubsystem.setSecondaryVelocity(IndexerConstants.LOWER_INDEXER_SPEED);
                        hopperSubsystem.setVelocity(HopperConstants.INDEXING_SPEED);
                    } else {
                        hopperSubsystem.setVelocity(HopperConstants.REVERSE_SPEED);
                    }
                },
                () -> {
//                    if (RobotContainer.areMechanismsExceptShooterAtGoalsAuto()) {
//                        indexerSubsystem.setVelocity(IndexerConstants.UPPER_INDEXER_SPEED);
//                        indexerSubsystem.setSecondaryVelocity(IndexerConstants.LOWER_INDEXER_SPEED);
//                        systemsOutOfToleranceCheck = 0;
//                    } else {
//                        systemsOutOfToleranceCheck++;
//                        if (systemsOutOfToleranceCheck > MAX_OUT_OF_TOLERANCE_TIMES) {
//                            indexerSubsystem.setVelocity(0);
//                            indexerSubsystem.setSecondaryVelocity(0);
//                        }
//                    }
//
//                    if (indexerSubsystem.isMechAtGoal(true) && indexerSubsystem.getGoal() != 0) {
//                        hopperSubsystem.setVelocity(HopperConstants.INDEXING_SPEED);
//                    } else {
//                        hopperSubsystem.setVelocity(-10);
//                    }

                    if (hopperSubsystem.getStatorCurrent() > 50d && Math.abs(hopperSubsystem.getAcceleration()) < 200) {
                        hopperStatorCurrentLimitCheck++;
                    } else hopperStatorCurrentLimitCheck = 0;

                    if (hopperStatorCurrentLimitCheck > STATOR_CURRENT_TIMES && hopperReverseCheck < MAX_HOPPER_REVERSAL_TIMES) {
                        if (shooterSubsystem.isMechAtGoal(true)) {
                            hopperSubsystem.setVelocity(HopperConstants.REVERSE_SPEED);
                            hopperReverseCheck++;
                        } else {
                            hopperStatorCurrentLimitCheck = 0;
                        }
                    } else {
                        hopperReverseCheck = 0;
                        if (RobotContainer.areMechanismsExceptShooterAtGoalsAuto()) {
                            indexerSubsystem.setVelocity(IndexerConstants.UPPER_INDEXER_SPEED);
                            indexerSubsystem.setSecondaryVelocity(IndexerConstants.LOWER_INDEXER_SPEED);
                            hopperSubsystem.setVelocity(HopperConstants.INDEXING_SPEED);
                            systemsOutOfToleranceCheck = 0;
                        } else {
                            systemsOutOfToleranceCheck++;
                            if (systemsOutOfToleranceCheck > MAX_OUT_OF_TOLERANCE_TIMES) {
                                hopperStatorCurrentLimitCheck = 0;
                                hopperSubsystem.setVelocity(HopperConstants.REVERSE_SPEED);
                            }
                        }
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

    public static Command agitateIntake() {
        return new FunctionalCommand(
                intakeTimer::restart,
                () -> {
                    if (!RobotContainer.isRightTriggerPressed() && intakeTimer.get() > INTAKE_AGITATION_INTERVAL) {
                        intakeTimer.reset();
                        if (isIntakeAgitationDown) {
                            intakePivotSubsystem.setPosition(IntakePivotConstants.UPAGITATE);
                            isIntakeAgitationDown = false;
                        } else {
                            intakePivotSubsystem.setPosition(IntakePivotConstants.DEPLOY);
                            isIntakeAgitationDown = true;
                        }
                    }

                },
                (_) -> {
                    intakePivotSubsystem.setPosition(IntakePivotConstants.DEPLOY);
                    intakeTimer.stop();
                    isIntakeAgitationDown = true;
                },
                () -> false,
                intakePivotSubsystem
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
