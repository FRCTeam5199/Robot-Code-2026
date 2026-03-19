// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import java.util.Map;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.ClimberConstants;
import frc.robot.constants.Constants;
import frc.robot.constants.HopperConstants;
import frc.robot.constants.IndexerConstants;
import frc.robot.constants.IntakePivotConstants;
import frc.robot.constants.IntakeRollerConstants;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakePivotSubsystem;
import frc.robot.subsystems.IntakeRollerSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.templates.HoodCommand;
import frc.robot.subsystems.templates.PositionCommand;
// import frc.robot.subsystems.templates.ShooterCommand;
//import frc.robot.subsystems.templates.ShooterCommand;
import frc.robot.subsystems.templates.TurretCommand;
import frc.robot.subsystems.templates.VelocityCommand;
import frc.robot.utility.ClimbMode;
import frc.robot.utility.ClimberSetpoint;
import frc.robot.utility.Setpoint;
import frc.robot.utility.ShotCalculator;
import frc.robot.utility.ShotMode;

public class

RobotContainer {
    public static final CommandXboxController commandXboxController = new CommandXboxController(Constants.XBOX_PORT);
//    public static final CommandXboxController operatorCommandXboxController
//            = new CommandXboxController(Constants.OPERATOR_XBOX_PORT);

    //Subsystems
    public static final CommandSwerveDrivetrain commandSwerveDrivetrain = TunerConstants.createDrivetrain();
    public static final Autos auton = Autos.getInstance();
    public static final IntakeRollerSubsystem intakeRollerSubsystem = IntakeRollerSubsystem.getInstance();
    public static final IntakePivotSubsystem intakePivotSubsystem = IntakePivotSubsystem.getInstance();
    public static final HopperSubsystem hopperSubsystem = HopperSubsystem.getInstance();
    public static final ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();
    public static final KickerSubsystem kickerSubsystem = KickerSubsystem.getInstance();
    public static final HoodSubsystem hoodSubsystem = HoodSubsystem.getInstance();
    public static final ShotCalculator shotCalculator = ShotCalculator.getInstance();
    public static final TurretSubsystem turretSubsystem = TurretSubsystem.getInstance();
    public static final IndexerSubsystem indexerSubsystem = IndexerSubsystem.getInstance();
    //    public static final ClimberSubsystem climberSubsystem = ClimberSubsystem.getInstance();
    public static final Vision vision = Vision.getInstance();

    //Intake Pivot
    public static final PositionCommand intakeStow = new PositionCommand(intakePivotSubsystem, IntakePivotConstants.STOW);
    public static final PositionCommand intakeDeploy = new PositionCommand(intakePivotSubsystem, IntakePivotConstants.DEPLOY);
    public static final PositionCommand intakeUpAgitate = new PositionCommand(intakePivotSubsystem, IntakePivotConstants.UPAGITATE);
    public static final PositionCommand intakeDownAgitate = new PositionCommand(intakePivotSubsystem, IntakePivotConstants.DOWNAGITATE);
    private static final Command intakeAgitation = new SequentialCommandGroup(intakeUpAgitate.withTimeout(.4), intakeDownAgitate.withTimeout(.4)).repeatedly();
    //Drive
    public static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric().withDesaturateWheelSpeeds(true)
            .withDeadband(Constants.MAX_SPEED * .05).withRotationalDeadband(Constants.MAX_ANGULAR_RATE * .05) // Add a 10% deadband
            .withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.OpenLoopVoltage);
    public static final ProfiledPIDController turnPIDController = new ProfiledPIDController(.05, 0.0, 0.0, new TrapezoidProfile.Constraints(100, 200));
    //Misc
    public static final Telemetry logger = new Telemetry(Constants.MAX_SPEED);
    private static final ProfiledPIDController drivePIDControllerX = new ProfiledPIDController(2, 0, 0, new TrapezoidProfile.Constraints(100, 200));
    private static final ProfiledPIDController drivePIDControllerXClose = new ProfiledPIDController(2, 0, 0.25, new TrapezoidProfile.Constraints(100, 200));
    //Turret Commands
    private static final TurretCommand turretControlAuto = new TurretCommand(turretSubsystem, 0, 0);
    private static final TurretCommand turretHub = new TurretCommand(turretSubsystem, Setpoint.HUB.getTurretAngle());
    private static final TurretCommand turretTower = new TurretCommand(turretSubsystem, Setpoint.TOWER.getTurretAngle());
    private static final TurretCommand turretLeftCorner = new TurretCommand(turretSubsystem, Setpoint.LEFT_CORNER.getTurretAngle());
    private static final TurretCommand turretOutpost = new TurretCommand(turretSubsystem, Setpoint.OUTPOST.getTurretAngle());
    //Hood Commands
    private static final HoodCommand hoodControlAuto = new HoodCommand(hoodSubsystem, 0, 0);
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
    private static final VelocityCommand shooterRevUp = new VelocityCommand(shooterSubsystem, shotCalculator.getShooterSpeed());
    //Kicker Commands
    private static final VelocityCommand kickerAuto = new VelocityCommand(kickerSubsystem, 0);
    private static final VelocityCommand kickerZero = new VelocityCommand(kickerSubsystem, 0);
    private static final VelocityCommand kickerHub = new VelocityCommand(kickerSubsystem, Setpoint.HUB.getKickerSpeed());
    private static final VelocityCommand kickerTower = new VelocityCommand(kickerSubsystem, Setpoint.TOWER.getKickerSpeed());
    private static final VelocityCommand kickerLeftCorner = new VelocityCommand(kickerSubsystem, Setpoint.LEFT_CORNER.getKickerSpeed());
    private static final VelocityCommand kickerOutpost = new VelocityCommand(kickerSubsystem, Setpoint.OUTPOST.getKickerSpeed());
    private static final VelocityCommand kickerRevUp = new VelocityCommand(kickerSubsystem, (shotCalculator.getShooterSpeed() * 5 / 3));
    //    Auton commands
    private static final Command revUp = new ParallelCommandGroup(shooterRevUp, kickerRevUp);
    //Indexer Commands
    private static final VelocityCommand indexerAuto = new VelocityCommand(indexerSubsystem, 0);
    private static final VelocityCommand indexerIdle = new VelocityCommand(indexerSubsystem, IndexerConstants.IDLING_SPEED);
    private static final VelocityCommand indexerIndex = new VelocityCommand(indexerSubsystem, IndexerConstants.INDEXING_SPEED);
    //Hopper Commands
    private static final VelocityCommand hopperAuto = new VelocityCommand(hopperSubsystem, 0);
    private static final VelocityCommand hopperIdle = new VelocityCommand(hopperSubsystem, HopperConstants.IDLING_SPEED);
    private static final VelocityCommand hopperIndex = new VelocityCommand(hopperSubsystem, HopperConstants.INDEXING_SPEED);
    //Intake Roller Commands
    private static final VelocityCommand intakeRollerStop = new VelocityCommand(intakeRollerSubsystem, 0);
    private static final VelocityCommand intakeRollerIntake = new VelocityCommand(intakeRollerSubsystem, IntakeRollerConstants.INTAKE_SPEED);
    private static final VelocityCommand intakeRollerIntakeAuton = new VelocityCommand(intakeRollerSubsystem, IntakeRollerConstants.INTAKE_SPEED);
    //Climber Commands
//    private static final PositionCommand climberClimb = new PositionCommand(climberSubsystem, ClimberConstants.CLIMB);
//    private static final PositionCommand climberDeploy = new PositionCommand(climberSubsystem, ClimberConstants.DEPLOY);
    public static double xVelocity = 0;
    public static double yVelocity = 0;
    public static double rotationVelocity = 0;
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
    private static Setpoint currentSetpoint = Setpoint.HUB;
    //Mode Commands
    private static final InstantCommand setHubSetpoint = new InstantCommand(() -> setCurrentSetpoint(Setpoint.HUB));
    private static final InstantCommand setTowerSetpoint = new InstantCommand(() -> setCurrentSetpoint(Setpoint.TOWER));
    private static final InstantCommand setLeftCornerSetpoint = new InstantCommand(() -> setCurrentSetpoint(Setpoint.LEFT_CORNER));
    private static final InstantCommand setOutpostSetpoint = new InstantCommand(() -> setCurrentSetpoint(Setpoint.OUTPOST));
    private static ClimberSetpoint climberSetpoint = ClimberSetpoint.CLIMB_LEFT_RED;
    private static boolean isAutonomous = false;
    //Button Bindings
    private static Command leftTriggerPressed;
    private static Command leftTriggerReleased;
    private static Command leftBumperPressed;
    private static Command leftBumperReleased;
    private static Command extend;
    private static Command retract;
    private static Command stop;
    private static ShotMode shotMode = ShotMode.SHOOTING;
    private static Translation2d[] robotCorners = new Translation2d[4];
    private static ClimbMode climbMode = ClimbMode.NOCLIMB;
    private static boolean filtersInitialized = false;


    public RobotContainer() {
        optimizeDrivetrain();
        leftTriggerPressed = RobotCommands.indexBallsAuto().alongWith(
                new ParallelCommandGroup(kickerAuto, shooterAuto,
                        hoodControlAuto, turretControlAuto));

        leftTriggerReleased = RobotCommands.idleState();

//        extend = RobotCommands.extendClimb();
//        retract = RobotCommands.retractClimb();
//        stop = RobotCommands.stopClimb();


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

        NamedCommands.registerCommand("shoot", leftTriggerPressed);
        NamedCommands.registerCommand("hoodZero", hoodZero);
        NamedCommands.registerCommand("revUp", revUp);
        NamedCommands.registerCommand("stable", leftTriggerReleased);
        NamedCommands.registerCommand("deployIntake", intakeDeploy);
        NamedCommands.registerCommand("stowIntake", intakeStow);
        NamedCommands.registerCommand("agitateIntake", intakeAgitation);
        NamedCommands.registerCommand("runIntake", intakeRollerIntake);
        NamedCommands.registerCommand("stopIntake", intakeRollerStop);
        NamedCommands.registerCommand("indexBalls", RobotCommands.indexBalls());

        NamedCommands.registerCommand("prepClimbLeft", Autos.pidAlignLeft());
//        NamedCommands.registerCommand("prepClimbRight", RobotCommands.prepAutoCLimbRight());
//        NamedCommands.registerCommand("climb", climberClimb);

        Autos.initializeAutos();
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

        if (currentState == null || currentState.Pose == null) return;

        velocity = Math.sqrt(Math.pow(RobotContainer.getSpeeds().vxMetersPerSecond, 2)
                + Math.pow(RobotContainer.getSpeeds().vyMetersPerSecond, 2));
        acceleration = (velocity - lastVelocity) / .02;
        lastVelocity = velocity;

        updateRobotCorners();

        // Sets Enums, default is Shooting
        // Shooting versus Shuttling depends on X, Shuttling left or right depends on Y
        if (getPose().getY() - Constants.RED_HUB_CENTER.getY() > 0) {
            shotMode = ShotMode.SHUTTLING_RIGHT;
        } else {
            shotMode = ShotMode.SHUTTLING_LEFT;
        }
        for (Translation2d robotCorner : robotCorners) {
            if (Robot.getAlliance() != null && Robot.getAlliance().equals(DriverStation.Alliance.Red)) {
                if (robotCorner.getX() - Constants.RED_HUB_FRONT_CENTER.getX() > .15)
                    shotMode = ShotMode.SHOOTING;
            } else {
                if (Constants.BLUE_HUB_FRONT_CENTER.getX() - robotCorner.getX() > .15)
                    shotMode = ShotMode.SHOOTING;
            }
        }
        double scalingFactor = 1.25;

        if (commandXboxController.getLeftY() < 0)
            requestXVelocity = -Math.pow(Math.abs(commandXboxController.getLeftY()), scalingFactor) * Constants.MAX_SPEED;
        else
            requestXVelocity = Math.pow(Math.abs(commandXboxController.getLeftY()), scalingFactor) * Constants.MAX_SPEED;

        if (commandXboxController.getLeftX() < 0)
            requestYVelocity = -Math.pow(Math.abs(commandXboxController.getLeftX()), scalingFactor) * Constants.MAX_SPEED;
        else
            requestYVelocity = Math.pow(Math.abs(commandXboxController.getLeftX()), scalingFactor) * Constants.MAX_SPEED;

        if (commandXboxController.getRightX() < 0)
            requestRotationalVelocity = Math.pow(Math.abs(commandXboxController.getRightX()), scalingFactor) * Constants.MAX_ANGULAR_RATE;
        else
            requestRotationalVelocity = -Math.pow(Math.abs(commandXboxController.getRightX()), scalingFactor) * Constants.MAX_ANGULAR_RATE;

//        requestXVelocity = commandXboxController.getLeftY() * Constants.MAX_SPEED;
//        requestYVelocity = commandXboxController.getLeftX() * Constants.MAX_SPEED;
//        requestRotationalVelocity = -commandXboxController.getRightX() * Constants.MAX_ANGULAR_RATE;

        // calculateAutoClimbVelocities();

        // Logging
        // logger.telemeterize(currentState);

//        System.out.println(shotCalculator.getTurretVelocity());
//        System.out.println(predictedWrapAround());
    }

    public static boolean areMechanismsAtGoalsAuto() {
        return turretSubsystem.isMechAtGoalAuto() && hoodSubsystem.isMechAtGoalAuto()
                && shooterSubsystem.isMechAtGoalAuto()
                && kickerSubsystem.isMechAtGoalAuto();
    }

    public static boolean areMechanismsAtGoals() {
        return turretSubsystem.isMechAtGoal() && hoodSubsystem.isMechAtGoal()
                && shooterSubsystem.isMechAtGoal(true)
                && kickerSubsystem.isMechAtGoal(true);
    }

//    public static Pose2d getPose() {
//        return currentState.Pose;
//    }

//    public static double get


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
        if (Robot.getAlliance().equals(DriverStation.Alliance.Red)) return -requestYVelocity;
        return requestYVelocity;
    }

    public static double getRequestXVelocity() {
        if (Robot.getAlliance().equals(DriverStation.Alliance.Red)) return -requestXVelocity;
        return requestXVelocity;
    }

    public static void calculateAutoClimbVelocities() {
//        double currentX = climberSubsystem.getDistance();
        double currentX = 0;

        double currentRotation = getPose().getRotation().getDegrees();
        double rotationalError;
        double goalRotation;
        if (Robot.getAlliance() != null && Robot.getAlliance().equals(DriverStation.Alliance.Red)) {
            goalRotation = 180d;
            rotationalError = goalRotation - currentRotation;
        } else {
            goalRotation = 0d;
            rotationalError = goalRotation - currentRotation;
        }

        while (rotationalError > 180) rotationalError -= 360;
        while (rotationalError < -180) rotationalError += 360;


        if (Robot.getAlliance() != null && Robot.getAlliance().equals(DriverStation.Alliance.Red)) {
            goalX = ClimberConstants.CLIMBING_X_DISTANCE_RED;
        } else {
            goalX = ClimberConstants.CLIMBING_X_DISTANCE_BLUE;
        }

        if ((Math.abs(currentX - goalX) > .15)) {
            xVelocity = drivePIDControllerX.calculate(currentX, goalX);
        } else {
            xVelocity = drivePIDControllerXClose.calculate(currentX, goalX);
        }

        if (xVelocity < 0) xVelocity -= Constants.DRIVE_X_KS;
        else xVelocity += Constants.DRIVE_X_KS;

        rotationVelocity = -turnPIDController.calculate(rotationalError, 0);

//        System.out.println("Rotation Velocity: " + rotationVelocity);
//        System.out.println("Goal Degrees: " + climberSetpoint.getPose2d().getRotation().getDegrees());

//        System.out.println(Math.abs(climberSubsystem.getDistance() - goalX));

        if (currentRotation > goalRotation) rotationVelocity -= .3;
        else rotationVelocity += .3;

//        System.out.println("X velocity: " + xVelocity);
//        System.out.println("Y velocity: " + yVelocity);
//        System.out.println("Rotational velocity: " + rotationVelocity);

//        System.out.println("Aligned X: " + (Math.abs(climberSubsystem.getDistance() - goalX) < .03
//                && Math.abs(getSpeeds().vxMetersPerSecond) < .01));
//        System.out.println("Speeds: " + getSpeeds().vxMetersPerSecond);
        // System.out.println("Mechs At Goal: " + areMechanismsAtGoalsAuto());
    }

    public static void setClimberSetpoint(ClimberSetpoint climberSetpoint) {
        RobotContainer.climberSetpoint = climberSetpoint;
    }

    public static double getXVelocity() {
        return xVelocity;
    }

    public static double getYVelocity() {
        return yVelocity;
    }

    public static double getRotationVelocity() {
        return rotationVelocity;
    }

    public static boolean alignedX() {
//        return Math.abs(climberSubsystem.getDistance() - goalX) < .03
//                && Math.abs(getSpeeds().vxMetersPerSecond) < .01;
        return false;
    }

    public static boolean alignedY() {
        return Math.abs(getSpeeds().vyMetersPerSecond) < .05;
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

    public static boolean isAutonomous() {
        return isAutonomous;
    }

    public static void setIsAutonomous(boolean isAutonomous) {
        RobotContainer.isAutonomous = isClimbing;
    }

    private void configureBindings() {
        commandSwerveDrivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
                commandSwerveDrivetrain.applyRequest(() -> drive.withVelocityX(-requestXVelocity) // Drive forward with negative Y (forward)
                        .withVelocityY(-requestYVelocity) // Drive left with negative X (left)
                        .withRotationalRate(requestRotationalVelocity) // Drive counterclockwise with negative X (left)
                ));

        // Field Centric
        commandXboxController.button(8).onTrue(commandSwerveDrivetrain
                .runOnce(commandSwerveDrivetrain::seedFieldCentric).alongWith(
                        new ConditionalCommand(
                                new InstantCommand(() -> commandSwerveDrivetrain.getPigeon2().setYaw(0)),
                                new InstantCommand(() -> commandSwerveDrivetrain.getPigeon2().setYaw(180)),
                                () -> Robot.getAlliance() == DriverStation.Alliance.Blue)
                ));

//        commandXboxController.x().onTrue(intakeDeploy);
//        commandXboxController.a().onTrue(intakeStow);

        commandXboxController.a().whileTrue(turretSubsystem.sysIdQuasistaticForward());
        commandXboxController.b().whileTrue(turretSubsystem.sysIdQuasistaticReverse());
        commandXboxController.y().whileTrue(turretSubsystem.sysIdDynamicForward());
        commandXboxController.x().whileTrue(turretSubsystem.sysIdDynamicReverse());


        commandXboxController.rightBumper().onTrue(intakeAgitation)
                .onFalse(new PositionCommand(intakePivotSubsystem, IntakePivotConstants.DEPLOY));

        commandXboxController.rightTrigger().onTrue(intakeRollerIntake)
                .onFalse(intakeRollerStop);

        commandXboxController.leftTrigger().onTrue(leftTriggerPressed)
                .onFalse(leftTriggerReleased);

        commandXboxController.leftBumper().onTrue(new VelocityCommand(intakeRollerSubsystem, -116))
                .onFalse(new VelocityCommand(intakeRollerSubsystem, 0));

//        operatorCommandXboxController.y().onTrue(setHubSetpoint);
//        operatorCommandXboxController.x().onTrue(setLeftCornerSetpoint);
//        operatorCommandXboxController.b().onTrue(setOutpostSetpoint);
//        operatorCommandXboxController.a().onTrue(setTowerSetpoint);

        // operatorCommandXboxController.povUp().onTrue(new InstantCommand(() -> shooterSubsystem.changeOffset(.5)));
        // operatorCommandXboxController.povDown().onTrue(new InstantCommand(() -> shooterSubsystem.changeOffset(-.5)));

        commandXboxController.povUp().onTrue(new InstantCommand(() -> shooterSubsystem.changeOffset(.5)));
        commandXboxController.povDown().onTrue(new InstantCommand(() -> shooterSubsystem.changeOffset(-.5)));

//        operatorCommandXboxController.rightBumper().onTrue(RobotCommands.outtake())
//                .onFalse(RobotCommands.idleState());
//        operatorCommandXboxController.leftBumper().onTrue(leftBumperPressed).onFalse(leftBumperReleased);
    }

    public Command getAutonomousCommand() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            return Autos.autonChooserRed.getSelected();
        } else {
            return Autos.autonChooserBlue.getSelected();
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
