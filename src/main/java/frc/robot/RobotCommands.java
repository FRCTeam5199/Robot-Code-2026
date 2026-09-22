package frc.robot;

import org.wpilib.command2.Command;
import org.wpilib.command2.FunctionalCommand;
import org.wpilib.command2.InstantCommand;
import org.wpilib.command2.ParallelCommandGroup;
import org.wpilib.command2.SequentialCommandGroup;
import org.wpilib.system.Timer;

import frc.robot.constants.HopperConstants;
import frc.robot.constants.KickerConstants;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakePivotSubsystem;
import frc.robot.subsystems.IntakeRollerSubsystem;
import frc.robot.subsystems.KickerSubsystem;
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
    private static final KickerSubsystem kickerSubsystem = KickerSubsystem.getInstance();
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
                        kickerSubsystem.setVelocity(KickerConstants.INDEXING_SPEED);
                    }
                    if (kickerSubsystem.isMechAtGoal(true) && kickerSubsystem.getGoal() != 0) {
                        hopperSubsystem.setVelocity(HopperConstants.INDEXING_SPEED);
                    } else {
                        hopperSubsystem.setVelocity(-10);
                    }
                },
                () -> {
                    if (RobotContainer.areMechanismsExceptShooterAtGoalsAuto()) {
                        kickerSubsystem.setVelocity(KickerConstants.INDEXING_SPEED);
                    } else {
                        kickerSubsystem.setVelocity(0);
                    }

                    if (kickerSubsystem.isMechAtGoal(true) && kickerSubsystem.getGoal() != 0) {
                        hopperSubsystem.setVelocity(HopperConstants.INDEXING_SPEED);
                    } else {
                        hopperSubsystem.setVelocity(-10);
                    }
                },
                (interrupted) -> {
                    kickerSubsystem.setVelocity(0);
                    hopperSubsystem.setVelocity(0);
                },
                () -> false,
                hopperSubsystem, kickerSubsystem
        );
    }

    public static Command indexBalls() {
        return new FunctionalCommand(
                () -> {
                    if (RobotContainer.areMechanismsAtGoalsAuto()) {
                        kickerSubsystem.setVelocity(KickerConstants.INDEXING_SPEED);
                    }
                    if (kickerSubsystem.isMechAtGoal(true) && kickerSubsystem.getGoal() != 0) {
                        hopperSubsystem.setVelocity(HopperConstants.INDEXING_SPEED);
                    }
                },
                () -> {
                    if (RobotContainer.areMechanismsExceptShooterAtGoals()) {
                        kickerSubsystem.setVelocity(KickerConstants.INDEXING_SPEED);
                    } else {
                        kickerSubsystem.setVelocity(0);
                    }

                    if (kickerSubsystem.isMechAtGoal(true) && kickerSubsystem.getGoal() != 0) {
                        hopperSubsystem.setVelocity(HopperConstants.INDEXING_SPEED);
                    } else {
                        hopperSubsystem.setVelocity(0);
                    }

                },
                (interrupted) -> {
                    kickerSubsystem.setVelocity(0);
                    hopperSubsystem.setVelocity(0);
                },
                () -> false,
                hopperSubsystem, kickerSubsystem
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
                new VelocityCommand(kickerSubsystem, 0)
        );
    }

    public static Command outtake() {
        return new ParallelCommandGroup(
                new VelocityCommand(hopperSubsystem, -50),
                new VelocityCommand(shooterSubsystem, -50),
                new VelocityCommand(kickerSubsystem, -50),
                new VelocityCommand(intakeRollerSubsystem, -50)
        );
    }
}
