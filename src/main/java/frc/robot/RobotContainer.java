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
import frc.robot.subsystems.IntakeLinearLeft;
import frc.robot.subsystems.IntakeLinearRight;
import frc.robot.subsystems.IntakeRollerSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.templates.PositionCommand;
import frc.robot.subsystems.templates.VelocityCommand;
import frc.robot.utility.Setpoint;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;

public class RobotContainer {
    public static final CommandXboxController commandXboxController = new CommandXboxController(Constants.XBOX_PORT);
    public static final CommandXboxController operatorXboxController = new CommandXboxController(Constants.OPERATOR_XBOX_PORT);
    public static int stallinL =0;
    public static int stallinR =0;
    public static int voltageTol = 5;

    //Subsystems
    public static final CommandSwerveDrivetrain commandSwerveDrivetrain = TunerConstants.createDrivetrain();
    public static final IntakeRollerSubsystem intakeRollerSubsystem = IntakeRollerSubsystem.getInstance();
    public static final IntakeLinearRight intakeLinearRightSubsystem = IntakeLinearRight.getInstance();
    public static final IntakeLinearLeft intakeLinearLeftSubsystem = IntakeLinearLeft.getInstance();

    public static final ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();
    public static final KickerSubsystem kickerSubsystem = KickerSubsystem.getInstance();
    public static final HoodSubsystem hoodSubsystem = HoodSubsystem.getInstance();

    //Intake Pivot
    public static final ParallelCommandGroup intakeStow = new ParallelCommandGroup(new PositionCommand(intakeLinearLeftSubsystem, IntakeLinearConstants.STOW), new PositionCommand(intakeLinearRightSubsystem, IntakeLinearConstants.STOW));
    public static final ParallelCommandGroup intakeDeploy = new ParallelCommandGroup(new PositionCommand(intakeLinearLeftSubsystem, IntakeLinearConstants.DEPLOY), new PositionCommand(intakeLinearRightSubsystem, IntakeLinearConstants.DEPLOY));

    //Drive
    public static double speedMultiplier = 1.0;
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
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        registerNamedCommands();
        Autos.initializeAutos();
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
        configureBindings();
    }

    public static boolean areMechanismsAtGoalsAuto(){
        return shooterSubsystem.isMechAtGoal(true) && hoodSubsystem.isMechAtGoal(false);
    }
    public static boolean areMechanismsAtGoalsExeptShooterAuto(){
        return hoodSubsystem.isMechAtGoal(false);
    }

    public static void periodic() {
        // System.out.println(areMechanismsAtGoalsAuto());
        stallinCount();
        double scalingFactor = 1.5;
        double rotationScalingFactor = 1.5;
        double maxSpeed = Constants.MAX_SPEED * speedMultiplier;

        if (commandXboxController.getLeftY() < 0)
            requestXVelocity = -Math.pow(Math.abs(commandXboxController.getLeftY()), scalingFactor) * maxSpeed;
        else
            requestXVelocity = Math.pow(Math.abs(commandXboxController.getLeftY()), scalingFactor) * maxSpeed;

        if (commandXboxController.getLeftX() < 0)
            requestYVelocity = -Math.pow(Math.abs(commandXboxController.getLeftX()), scalingFactor) * maxSpeed;
        else
            requestYVelocity = Math.pow(Math.abs(commandXboxController.getLeftX()), scalingFactor) * maxSpeed;

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
                                RobotCommands.indexBallsAuto())),
                        Map.entry(Setpoint.SHUTTLELONG, new ParallelCommandGroup(
                                new VelocityCommand(shooterSubsystem, Setpoint.SHUTTLELONG.getShooterSpeed()),
                                new PositionCommand(hoodSubsystem, Setpoint.SHUTTLELONG.getHoodAngle()),
                                RobotCommands.indexBallsAuto())),
                        Map.entry(Setpoint.CHRISLAN, new ParallelCommandGroup(
                                new VelocityCommand(shooterSubsystem, Setpoint.CHRISLAN.getShooterSpeed()),
                                new PositionCommand(hoodSubsystem, Setpoint.CHRISLAN.getHoodAngle()),
                                RobotCommands.indexBallsAuto())),
                        Map.entry(Setpoint.SHUTTLESHORT, new ParallelCommandGroup(
                            new VelocityCommand(shooterSubsystem, Setpoint.SHUTTLESHORT.getShooterSpeed()),
                                new PositionCommand(hoodSubsystem, Setpoint.SHUTTLESHORT.getHoodAngle())
                                .andThen(RobotCommands.indexBallsAuto())))),
                RobotContainer::getSetpoint))
        .onFalse(
                new ParallelCommandGroup(
                        new VelocityCommand(kickerSubsystem, KickerConstants.KICKER_REVERSE),
                        new VelocityCommand(shooterSubsystem, 0),
                        new PositionCommand(hoodSubsystem, HoodConstants.MIN)));

        // speed limiter
        commandXboxController.b().toggleOnTrue(Commands.startEnd(
            () -> speedMultiplier = 0.75, 
            () -> speedMultiplier = 1.0));

        commandXboxController.y().onTrue(RobotCommands.intakeOut());
        commandXboxController.a().onTrue(intakeStow);

        commandXboxController.leftTrigger().onTrue(intakeRollerIntake).onFalse(intakeRollerStop);
        commandXboxController.rightBumper().onTrue(intakeRollerOuttake).onFalse(intakeRollerStop);

        // X-drive
        commandXboxController.rightTrigger().whileTrue(commandSwerveDrivetrain.applyRequest(() -> brake));

        operatorXboxController.y().onTrue(new InstantCommand(() -> setSetpoint(Setpoint.SHUTTLELONG)));
        operatorXboxController.x().onTrue(new InstantCommand(() -> setSetpoint(Setpoint.SHUTTLESHORT)));
        operatorXboxController.a().onTrue(new InstantCommand(() -> setSetpoint(Setpoint.TOWER)));
        operatorXboxController.b().onTrue(new InstantCommand(() -> setSetpoint(Setpoint.CHRISLAN)));

        // operatorXboxController.x().onTrue(new InstantCommand(()->setSetpoint(Setpoint.)));
        // operatorXboxController.b().onTrue(new InstantCommand(()->setSetpoint(Setpoint.)));
    }

    private void registerNamedCommands() {
         NamedCommands.registerCommand("ShootTower", new ParallelCommandGroup(
                new VelocityCommand(shooterSubsystem, Setpoint.TOWER.getShooterSpeed()),
                new PositionCommand(hoodSubsystem, Setpoint.TOWER.getHoodAngle()),
                new VelocityCommand(kickerSubsystem, KickerConstants.KICKER_SPEED)));
        NamedCommands.registerCommand("ShooterStop", new ParallelCommandGroup(
                new VelocityCommand(kickerSubsystem, KickerConstants.KICKER_REVERSE),
                new VelocityCommand(shooterSubsystem, 0),
                new PositionCommand(hoodSubsystem, HoodConstants.MIN)));
    }

    public Command getAutonomousCommand() {
       return autoChooser.getSelected();
    }

    private static void stallinLeft(){
        if(intakeLinearLeftSubsystem.getMotorVoltage()>=voltageTol) {
            stallinL ++;
        }else {stallinL =0;};
    }

    private static void stallinRight(){
        if(intakeLinearLeftSubsystem.getMotorVoltage()>=voltageTol) {
            stallinR ++;
        }else {stallinR = 0;};
    }

    public static void stallinCount() {
        stallinLeft();
        stallinRight();
    }

    public static int getStallinL() {
        return stallinL;
    }
    public static int getStallinR() {
        return stallinR;
    }

    public void optimizeDrivetrain() {
        for (int i = 0; i < 4; i++) {
            commandSwerveDrivetrain.getModules()[i].getDriveMotor().optimizeBusUtilization();
            commandSwerveDrivetrain.getModules()[i].getSteerMotor().optimizeBusUtilization();
            commandSwerveDrivetrain.getModules()[i].getEncoder().optimizeBusUtilization();
        }
    }
}
