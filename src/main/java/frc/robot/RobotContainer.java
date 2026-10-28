// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.Constants;
import frc.robot.constants.HoodConstants;
import frc.robot.constants.IntakeLinearConstants;
import frc.robot.constants.IntakeRollerConstants;
import frc.robot.constants.KickerConstants;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IntakeLinearSubsystem;
import frc.robot.subsystems.IntakeRollerSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.templates.PositionCommand;
import frc.robot.subsystems.templates.VelocityCommand;
import frc.robot.utility.Setpoint;

public class RobotContainer {
    public static final CommandXboxController commandXboxController = new CommandXboxController(Constants.XBOX_PORT);
    public static final CommandXboxController operatorXboxController = new CommandXboxController(Constants.OPERATOR_XBOX_PORT);

    //Subsystems
    public static final CommandSwerveDrivetrain commandSwerveDrivetrain = TunerConstants.createDrivetrain();
    public static final IntakeRollerSubsystem intakeRollerSubsystem = IntakeRollerSubsystem.getInstance();
    public static final IntakeLinearSubsystem intakeLinearSubsystem = IntakeLinearSubsystem.getInstance();
    public static final ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();
    public static final KickerSubsystem kickerSubsystem = KickerSubsystem.getInstance();
    public static final HoodSubsystem hoodSubsystem = HoodSubsystem.getInstance();
    //Intake Pivot
    public static final PositionCommand intakeStow = new PositionCommand(intakeLinearSubsystem, IntakeLinearConstants.STOW);
    public static final PositionCommand intakeDeploy = new PositionCommand(intakeLinearSubsystem, IntakeLinearConstants.DEPLOY, false);
    //Drive
    public static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric().withDesaturateWheelSpeeds(true)
            .withDeadband(Constants.MAX_SPEED * .05).withRotationalDeadband(Constants.MAX_ANGULAR_RATE * .05)
            .withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.OpenLoopVoltage);
    //Intake Roller Commands
    private static final VelocityCommand intakeRollerStop = new VelocityCommand(intakeRollerSubsystem, 0);
    private static final VelocityCommand intakeRollerIntake = new VelocityCommand(intakeRollerSubsystem, IntakeRollerConstants.INTAKE_SPEED);
    private static final VelocityCommand intakeRollerOuttake = new VelocityCommand(intakeRollerSubsystem, IntakeRollerConstants.OUTTAKE_SPEED);
    public static Setpoint currentSetpoint = Setpoint.TOWER;
    public static double shooterTower = 45;
    public static double hoodTower = 12;
    public static double shooterTrench = 40;
    public static double hoodTrench = 12;
    public static double shooterShuttle = 60;
    //Current Robot State
    public static SwerveDrivetrain.SwerveDriveState currentState;
    public static Pose2d filteredPose;
    public static double requestXVelocity;
    public static double requestYVelocity;
    public static double requestRotationalVelocity;
    public static double lastVelocity = 0, velocity = 0;
    public static double acceleration = 0;
    public static boolean isClimbing = false;
    public static boolean isClimberRetracting = false;
    public static double goalX;
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    public RobotContainer() {
        Autos.initializeAutos();
        configureBindings();
    }

    public static void periodic() {
        double scalingFactor = 1.5;
        double rotationScalingFactor = 1.5;

        if (commandXboxController.getLeftY() < 0)
            requestXVelocity = -Math.pow(Math.abs(commandXboxController.getLeftY()), scalingFactor) * Constants.MAX_SPEED;
        else
            requestXVelocity = Math.pow(Math.abs(commandXboxController.getLeftY()), scalingFactor) * Constants.MAX_SPEED;

        if (commandXboxController.getLeftX() < 0)
            requestYVelocity = -Math.pow(Math.abs(commandXboxController.getLeftX()), scalingFactor) * Constants.MAX_SPEED;
        else
            requestYVelocity = Math.pow(Math.abs(commandXboxController.getLeftX()), scalingFactor) * Constants.MAX_SPEED;

        if (commandXboxController.getRightX() < 0)
            requestRotationalVelocity = Math.pow(Math.abs(commandXboxController.getRightX()), rotationScalingFactor) * Constants.MAX_ANGULAR_RATE;
        else
            requestRotationalVelocity = -Math.pow(Math.abs(commandXboxController.getRightX()), rotationScalingFactor) * Constants.MAX_ANGULAR_RATE;
    }

    public static ChassisSpeeds getSpeeds() {
        return currentState.Speeds;
    }

    public static double getRequestRotationalVelocity() {
        return requestRotationalVelocity;
    }

    public static double getRequestYVelocity() {
        if (Robot.getAlliance().equals(DriverStation.Alliance.Red)) return -requestYVelocity;
        return requestYVelocity;
    }

    public static double getRequestXVelocity() {
        if (Robot.getAlliance().equals(DriverStation.Alliance.Red)) return -requestXVelocity;
        return requestXVelocity;
    }

    public static Pose2d getPose() {
        return currentState.Pose;
    }

    public static boolean isClimbing() {
        return isClimbing;
    }

    public static void setIsClimbing(boolean isClimbing) {
        RobotContainer.isClimbing = isClimbing;
    }

    public static Setpoint getSetpoint() {
        return currentSetpoint;
    }

    public static void setSetpoint(Setpoint setpoint) {
        currentSetpoint = setpoint;
    }

    private void configureBindings() {
        commandSwerveDrivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
                commandSwerveDrivetrain.applyRequest(() -> drive.withVelocityX(-requestXVelocity) // Drive forward with negative Y (forward)
                        .withVelocityY(-requestYVelocity) // Drive left with negative X (left)
                        .withRotationalRate(requestRotationalVelocity))); // Drive counterclockwise with negative X (left));

        // Field Centric
        commandXboxController.button(8).onTrue(commandSwerveDrivetrain
                .runOnce(commandSwerveDrivetrain::seedFieldCentric).alongWith(
                        new ConditionalCommand(
                                new InstantCommand(() -> commandSwerveDrivetrain.getPigeon2().setYaw(180)),
                                new InstantCommand(() -> commandSwerveDrivetrain.getPigeon2().setYaw(0)),
                                () -> Robot.getAlliance() == DriverStation.Alliance.Blue)
                ));

         commandXboxController.rightTrigger().onTrue(new SelectCommand<>(
                Map.ofEntries(
                        Map.entry(Setpoint.TOWER, new ParallelCommandGroup(
                                new VelocityCommand(shooterSubsystem, Setpoint.TOWER.getShooterSpeed()),
                                new PositionCommand(hoodSubsystem, Setpoint.TOWER.getHoodAngle()),
                                new VelocityCommand(kickerSubsystem, KickerConstants.KICKER_SPEED))),
                        Map.entry(Setpoint.SHUTTLE, new ParallelCommandGroup(
                                new VelocityCommand(shooterSubsystem, Setpoint.SHUTTLE.getShooterSpeed()),
                                new PositionCommand(hoodSubsystem, Setpoint.SHUTTLE.getHoodAngle()),
                                new VelocityCommand(kickerSubsystem, KickerConstants.KICKER_SPEED)))),
                RobotContainer::getSetpoint
        )).onFalse(
                new ParallelCommandGroup(
                        new VelocityCommand(kickerSubsystem, KickerConstants.KICKER_REVERSE),
                        new VelocityCommand(shooterSubsystem, 0),
                        new PositionCommand(hoodSubsystem, HoodConstants.MIN)));

        commandXboxController.y().onTrue(intakeDeploy).onTrue(new InstantCommand(() -> intakeLinearSubsystem.setBrake(false)));
        commandXboxController.a().onTrue(intakeStow).onTrue(new InstantCommand(() -> intakeLinearSubsystem.setBrake(true)));

        commandXboxController.leftTrigger().onTrue(intakeRollerIntake).onFalse(intakeRollerStop);
        commandXboxController.rightBumper().onTrue(intakeRollerOuttake).onFalse(intakeRollerStop);

        // X-drive
        commandXboxController.rightTrigger().whileTrue(commandSwerveDrivetrain.applyRequest(() -> brake));

        operatorXboxController.y().onTrue(new InstantCommand(() -> setSetpoint(Setpoint.SHUTTLE)));
        operatorXboxController.a().onTrue(new InstantCommand(() -> setSetpoint(Setpoint.TOWER)));

        // operatorXboxController.x().onTrue(new InstantCommand(()->setSetpoint(Setpoint.)));
        // operatorXboxController.b().onTrue(new InstantCommand(()->setSetpoint(Setpoint.)));
    }

    public Command getAutonomousCommand() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            return new PathPlannerAuto("Top");
        } else {
            return new PathPlannerAuto("Top");
        }
    }

    public void optimizeDrivetrain() {
        for (int i = 0; i < 4; i++) {
            commandSwerveDrivetrain.getModules()[i].getDriveMotor().optimizeBusUtilization();
            commandSwerveDrivetrain.getModules()[i].getSteerMotor().optimizeBusUtilization();
            commandSwerveDrivetrain.getModules()[i].getEncoder().optimizeBusUtilization();
        }
    }
}
