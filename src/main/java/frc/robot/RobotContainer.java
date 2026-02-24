// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import java.util.Map;
import java.util.Set;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.templates.HoodCommand;
import frc.robot.subsystems.templates.PositionCommand;
// import frc.robot.subsystems.templates.ShooterCommand;
//import frc.robot.subsystems.templates.ShooterCommand;
import frc.robot.subsystems.templates.TurretCommand;
import frc.robot.subsystems.templates.VelocityCommand;
import frc.robot.utility.Setpoint;
import frc.robot.utility.ShotCalculator;
import frc.robot.utility.ShotMode;

public class RobotContainer {
    public static final CommandXboxController commandXboxController = new CommandXboxController(Constants.XBOX_PORT);

    //Subsystems
    public static final CommandSwerveDrivetrain commandSwerveDrivetrain = TunerConstants.createDrivetrain();
    public static final IntakeRollerSubsystem intakeRollerSubsystem = IntakeRollerSubsystem.getInstance();
    public static final IntakePivotSubsystem intakePivotSubsystem = IntakePivotSubsystem.getInstance();
    public static final HopperSubsystem hopperSubsystem = HopperSubsystem.getInstance();
    public static final ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();
    public static final KickerSubsystem kickerSubsystem = KickerSubsystem.getInstance();
    public static final HoodSubsystem hoodSubsystem = HoodSubsystem.getInstance();
    public static final ShotCalculator shotCalculator = ShotCalculator.getInstance();
    public static final TurretSubsystem turretSubsystem = TurretSubsystem.getInstance();
    public static final IndexerSubsystem indexerSubsystem = IndexerSubsystem.getInstance();
    public static final Vision vision = Vision.getInstance();

    //Turret Commands
    private static final TurretCommand turretControlAuto = new TurretCommand(turretSubsystem, 0, 0);
    private static final TurretCommand turretHub = new TurretCommand(turretSubsystem, Setpoint.HUB.getTurretAngle());
    private static final TurretCommand turretTower = new TurretCommand(turretSubsystem, Setpoint.TOWER.getTurretAngle());
    private static final TurretCommand turretLeftCorner = new TurretCommand(turretSubsystem, Setpoint.LEFT_CORNER.getTurretAngle());
    private static final TurretCommand turretOutpost = new TurretCommand(turretSubsystem, Setpoint.OUTPOST.getTurretAngle());
    //Hood Commands
    private static final HoodCommand hoodControlAuto = new HoodCommand(hoodSubsystem, 0, 0);
    //        private static PositionCommand hoodControlAuto = new PositionCommand(hoodSubsystem, 0, true, false, 0);
    private static final HoodCommand hoodZero = new HoodCommand(hoodSubsystem, 0);
    private static final HoodCommand hoodHub = new HoodCommand(hoodSubsystem, Setpoint.HUB.getHoodAngle());
    private static final HoodCommand hoodTower = new HoodCommand(hoodSubsystem, Setpoint.TOWER.getHoodAngle());
    private static final HoodCommand hoodLeftCorner = new HoodCommand(hoodSubsystem, Setpoint.LEFT_CORNER.getHoodAngle());
    private static final HoodCommand hoodOutpost = new HoodCommand(hoodSubsystem, Setpoint.OUTPOST.getHoodAngle());

    //Shooter Commands
    private static final VelocityCommand shooterAuto = new VelocityCommand(shooterSubsystem, 0);
    private static final VelocityCommand shooterStop = new VelocityCommand(shooterSubsystem, 0);
    private static final VelocityCommand shooterHub = new VelocityCommand(shooterSubsystem, Setpoint.HUB.getShooterSpeed());
    private static final VelocityCommand shooterTower = new VelocityCommand(shooterSubsystem, Setpoint.TOWER.getShooterSpeed());
    private static final VelocityCommand shooterLeftCorner = new VelocityCommand(shooterSubsystem, Setpoint.LEFT_CORNER.getShooterSpeed());
    private static final VelocityCommand shooterOutpost = new VelocityCommand(shooterSubsystem, Setpoint.OUTPOST.getShooterSpeed());

    //Kicker Commands
    private static final VelocityCommand kickerAuto = new VelocityCommand(kickerSubsystem, 0);
    private static final VelocityCommand kickerZero = new VelocityCommand(kickerSubsystem, 0);
    private static final VelocityCommand kickerHub = new VelocityCommand(kickerSubsystem, Setpoint.HUB.getKickerSpeed());
    private static final VelocityCommand kickerTower = new VelocityCommand(kickerSubsystem, Setpoint.TOWER.getKickerSpeed());
    private static final VelocityCommand kickerLeftCorner = new VelocityCommand(kickerSubsystem, Setpoint.LEFT_CORNER.getKickerSpeed());
    private static final VelocityCommand kickerOutpost = new VelocityCommand(kickerSubsystem, Setpoint.OUTPOST.getKickerSpeed());

    //Indexer Commands
    private static final VelocityCommand indexerIdle = new VelocityCommand(indexerSubsystem, IndexerConstants.IDLING_SPEED);
    private static final VelocityCommand indexerIndex = new VelocityCommand(indexerSubsystem, IndexerConstants.INDEXING_SPEED);

    //Hopper Commands
    private static final VelocityCommand hopperIdle = new VelocityCommand(hopperSubsystem, HopperConstants.IDLING_SPEED);
    private static final VelocityCommand hopperIndex = new VelocityCommand(hopperSubsystem, HopperConstants.INDEXING_SPEED);

    //Intake Pivot
    public static final PositionCommand intakeStow = new PositionCommand(intakePivotSubsystem, IntakePivotConstants.STOW);
    public static final PositionCommand intakeDeploy = new PositionCommand(intakePivotSubsystem, IntakePivotConstants.DEPLOY);

    //Intake Roller Commands
    private static final VelocityCommand intakeRollerStop = new VelocityCommand(intakeRollerSubsystem, 0);
    private static final VelocityCommand intakeRollerIntake = new VelocityCommand(intakeRollerSubsystem, IntakeRollerConstants.INTAKE_SPEED);

    //Mode Commands
    private static final InstantCommand setHubSetpoint = new InstantCommand(() -> setCurrentSetpoint(Setpoint.HUB));
    private static final InstantCommand setTowerSetpoint = new InstantCommand(() -> setCurrentSetpoint(Setpoint.TOWER));
    private static final InstantCommand setLeftCornerSetpoint = new InstantCommand(() -> setCurrentSetpoint(Setpoint.LEFT_CORNER));
    private static final InstantCommand setOutpostSetpoint = new InstantCommand(() -> setCurrentSetpoint(Setpoint.OUTPOST));

    //Button Bindings
    private static Command leftTriggerPressed;
    private static Command leftTriggerReleased;
    private static Command leftBumperPressed;
    private static Command leftBumperReleased;

    //Drive
    public static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric().withDesaturateWheelSpeeds(true)
            .withDeadband(Constants.MAX_SPEED * .05).withRotationalDeadband(Constants.MAX_ANGULAR_RATE * .05) // Add a 10% deadband
            .withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.OpenLoopVoltage);

    //Current Robot State
    public static SwerveDrivetrain.SwerveDriveState currentState;
    private static Setpoint currentSetpoint = Setpoint.HUB;
    private static ShotMode shotMode = ShotMode.SHOOTING;
    private static Translation2d[] robotCorners = new Translation2d[4];

    //Misc
    public static final Telemetry logger = new Telemetry(Constants.MAX_SPEED);
    public static double requestXVelocity;
    public static double requestYVelocity;
    public static double requestRotationalVelocity;

    public RobotContainer() {
        leftTriggerPressed = RobotCommands.indexBalls().alongWith(
                new ParallelCommandGroup(shooterAuto, kickerAuto, turretControlAuto));
        leftTriggerReleased = RobotCommands.idleState();

        leftBumperPressed = new SelectCommand<>(Map.ofEntries(
                Map.entry(Setpoint.HUB, new ParallelCommandGroup(
                        turretHub, hoodHub, shooterHub, kickerHub
                )),
                Map.entry(Setpoint.TOWER, new ParallelCommandGroup(
                        turretTower, hoodTower, shooterTower, kickerTower
                )),
                Map.entry(Setpoint.OUTPOST, new ParallelCommandGroup(
                        turretOutpost, hoodOutpost, shooterOutpost, kickerOutpost
                )),
                Map.entry(Setpoint.LEFT_CORNER, new ParallelCommandGroup(
                        turretLeftCorner, hoodLeftCorner, shooterLeftCorner, kickerLeftCorner
                ))
        ), RobotContainer::getCurrentSetpoint).alongWith(RobotCommands.indexBalls());
        leftBumperReleased = RobotCommands.idleState();

        configureBindings();
    }

    public static void updateRobotCorners() {
        robotCorners[0] = getPose().transformBy(Constants.FRONT_LEFT_CORNER).getTranslation();
        robotCorners[1] = getPose().transformBy(Constants.FRONT_RIGHT_CORNER).getTranslation();
        robotCorners[2] = getPose().transformBy(Constants.BACK_LEFT_CORNER).getTranslation();
        robotCorners[3] = getPose().transformBy(Constants.BACK_RIGHT_CORNER).getTranslation();
    }

    public static void periodic() {
        currentState = commandSwerveDrivetrain.getStateCopy();

        updateRobotCorners();

        // Sets Enums, default is Shooting
        // Shooting versus Shuttling depends on X, Shuttling left or right depends on Y
        shotMode = ShotMode.SHOOTING;
        for (Translation2d robotCorner : robotCorners) {
            if (Constants.TRENCH.getX() - robotCorner.getX() > .05) {
                if (getPose().getY() - Constants.RED_HUB_CENTER.getY() > 0) {
                    shotMode = ShotMode.SHUTTLING_RIGHT;
                } else {
                    shotMode = ShotMode.SHUTTLING_LEFT;
                }
                break;
            }
        }

        requestXVelocity = -commandXboxController.getLeftY() * Constants.MAX_SPEED;
        requestYVelocity = -commandXboxController.getLeftX() * Constants.MAX_SPEED;
        requestRotationalVelocity = -commandXboxController.getRightX() * Constants.MAX_SPEED;

        // Logging
        logger.telemeterize(currentState);
    }

    private void configureBindings() {
        commandSwerveDrivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
                commandSwerveDrivetrain.applyRequest(() -> drive.withVelocityX(-commandXboxController.getLeftY() * Constants.MAX_SPEED) // Drive forward with negative Y (forward)
                        .withVelocityY(-commandXboxController.getLeftX() * Constants.MAX_SPEED) // Drive left with negative X (left)
                        .withRotationalRate(-commandXboxController.getRightX() * Constants.MAX_ANGULAR_RATE) // Drive counterclockwise with negative X (left)
                ));
        // Field Centric
        commandXboxController.button(8).onTrue(commandSwerveDrivetrain
                .runOnce(commandSwerveDrivetrain::seedFieldCentric));
        // Zeroing Pigeon (Always towards red alliance)
        commandXboxController.button(7)
                .onTrue(new InstantCommand(() -> commandSwerveDrivetrain.getPigeon2().setYaw(0)));

        commandXboxController.povDown().onTrue(intakeDeploy);
        commandXboxController.povUp().onTrue(intakeStow);

        commandXboxController.rightTrigger().onTrue(intakeRollerIntake)
                .onFalse(intakeRollerStop);
//        commandXboxController.rightTrigger()
//                .onTrue(new ParallelCommandGroup(intakeDeploy, intakeRollerIntake))
//                .onFalse(new ParallelCommandGroup(intakeStow, intakeRollerStop));

        commandXboxController.leftTrigger().onTrue(leftTriggerPressed)
                .onFalse(leftTriggerReleased);

        // Back Up Setpoints
        commandXboxController.leftBumper().onTrue(leftBumperPressed)
                .onFalse(leftBumperReleased);


        commandXboxController.y().onTrue(setHubSetpoint);
        commandXboxController.x().onTrue(setLeftCornerSetpoint);
        commandXboxController.b().onTrue(setOutpostSetpoint);
        commandXboxController.a().onTrue(setTowerSetpoint);

        // commandXboxController.a().onTrue(
        //         new SequentialCommandGroup(
        //                 new PositionCommand(intakePivotSubsystem, IntakePivotConstants.DEPLOY),
        //                 new PositionCommand(intakePivotSubsystem, 80)
        //         ).repeatedly()
        // );
        commandXboxController.povLeft().onTrue(RobotCommands.zeroTurret());

    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }

    public static boolean areMechanismsAtGoals() {
        return turretSubsystem.isMechAtGoal() && hoodSubsystem.isMechAtGoal(false)
                && shooterSubsystem.isMechAtGoal(true)
                && kickerSubsystem.isMechAtGoal(true);
    }

    public static Pose2d getPose() {
        return currentState.Pose;
    }

    public static ChassisSpeeds getSpeeds() {
        return currentState.Speeds;
    }

    public static ShotMode getShotMode() {
        return shotMode;
    }

    public static void setShotMode(ShotMode shotMode) {
        RobotContainer.shotMode = shotMode;
    }

    public static Setpoint getCurrentSetpoint() {
        return currentSetpoint;
    }

    public static void setCurrentSetpoint(Setpoint currentSetpoint) {
        RobotContainer.currentSetpoint = currentSetpoint;
    }

    public static TurretCommand getTurretControlAuto() {
        return turretControlAuto;
    }

    public static HoodCommand getHoodControlAuto() {
        return hoodControlAuto;
    }

    public static VelocityCommand getShooterControlAuto() {
        return shooterAuto;
    }

    public static VelocityCommand getKickerControlAuto() {
        return kickerAuto;
    }

    public static double getRequestRotationalVelocity() {
        return requestRotationalVelocity;
    }

    public static double getRequestYVelocity() {
        return requestYVelocity;
    }

    public static double getRequestXVelocity() {
        return requestXVelocity;
    }
}
