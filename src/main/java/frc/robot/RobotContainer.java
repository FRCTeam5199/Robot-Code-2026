// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import java.util.Map;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.templates.HoodCommand;
import frc.robot.subsystems.templates.PositionCommand;
// import frc.robot.subsystems.templates.ShooterCommand;
//import frc.robot.subsystems.templates.ShooterCommand;
import frc.robot.subsystems.templates.TurretCommand;
import frc.robot.subsystems.templates.VelocityCommand;
import frc.robot.utility.*;

public class RobotContainer {
    public static final CommandXboxController commandXboxController = new CommandXboxController(Constants.XBOX_PORT);

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
    public static final ClimberSubsystem climberSubsystem = ClimberSubsystem.getInstance();
    public static final Vision vision = Vision.getInstance();

    //Intake Pivot
    public static final PositionCommand intakeStow = new PositionCommand(intakePivotSubsystem, IntakePivotConstants.STOW);
    public static final PositionCommand intakeDeploy = new PositionCommand(intakePivotSubsystem, IntakePivotConstants.DEPLOY);

    //Drive
    public static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric().withDesaturateWheelSpeeds(true)
            .withDeadband(Constants.MAX_SPEED * .05).withRotationalDeadband(Constants.MAX_ANGULAR_RATE * .05) // Add a 10% deadband
            .withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.OpenLoopVoltage);
    private static final ProfiledPIDController drivePIDControllerX = new ProfiledPIDController(2, 0, 0, new TrapezoidProfile.Constraints(100, 200));
    private static final ProfiledPIDController drivePIDControllerXClose = new ProfiledPIDController(4, 0, 0, new TrapezoidProfile.Constraints(100, 200));

    private static final ProfiledPIDController drivePIDControllerY = new ProfiledPIDController(2.5, 0, 0, new TrapezoidProfile.Constraints(100, 200));
    private static final ProfiledPIDController drivePIDControllerYClose = new ProfiledPIDController(4.5, 0.0, 0, new TrapezoidProfile.Constraints(100, 200));

    public static final ProfiledPIDController turnPIDController = new ProfiledPIDController(.05, 0.0, 0.0, new TrapezoidProfile.Constraints(100, 200));

    public static double xVelocity = 0;
    public static double yVelocity = 0;
    public static double rotationVelocity = 0;

    //Misc
    public static final Telemetry logger = new Telemetry(Constants.MAX_SPEED);

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

    //Kicker Commands
    private static final VelocityCommand kickerAuto = new VelocityCommand(kickerSubsystem, 0);
    private static final VelocityCommand kickerZero = new VelocityCommand(kickerSubsystem, 0);
    private static final VelocityCommand kickerHub = new VelocityCommand(kickerSubsystem, Setpoint.HUB.getKickerSpeed());
    private static final VelocityCommand kickerTower = new VelocityCommand(kickerSubsystem, Setpoint.TOWER.getKickerSpeed());
    private static final VelocityCommand kickerLeftCorner = new VelocityCommand(kickerSubsystem, Setpoint.LEFT_CORNER.getKickerSpeed());
    private static final VelocityCommand kickerOutpost = new VelocityCommand(kickerSubsystem, Setpoint.OUTPOST.getKickerSpeed());

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

    //Current Robot State
    public static SwerveDrivetrain.SwerveDriveState currentState;
    private static Setpoint currentSetpoint = Setpoint.HUB;
    private static ClimberSetpoint climberSetpoint = ClimberSetpoint.CLIMB_LEFT_RED;
    public static double requestXVelocity;
    public static double requestYVelocity;
    public static double requestRotationalVelocity;
    public static double lastVelocity = 0, velocity = 0;
    public static double acceleration = 0;

    //Button Bindings
    private static Command leftTriggerPressed;
    private static Command leftTriggerReleased;
    private static Command leftBumperPressed;
    private static Command leftBumperReleased;
    private static Command extend;
    private static Command retract;
    private static Command stop;

    //Mode Commands
    private static final InstantCommand setHubSetpoint = new InstantCommand(() -> setCurrentSetpoint(Setpoint.HUB));
    private static final InstantCommand setTowerSetpoint = new InstantCommand(() -> setCurrentSetpoint(Setpoint.TOWER));
    private static final InstantCommand setLeftCornerSetpoint = new InstantCommand(() -> setCurrentSetpoint(Setpoint.LEFT_CORNER));
    private static final InstantCommand setOutpostSetpoint = new InstantCommand(() -> setCurrentSetpoint(Setpoint.OUTPOST));
    private static ShotMode shotMode = ShotMode.SHOOTING;
    private static Translation2d[] robotCorners = new Translation2d[4];
    private static ClimbMode climbMode = ClimbMode.NOCLIMB;

    public RobotContainer() {
        leftTriggerPressed = RobotCommands.indexBallsAuto().alongWith(
                new ParallelCommandGroup(kickerAuto, shooterAuto, hoodControlAuto, turretControlAuto));
        leftTriggerReleased = RobotCommands.idleState();

        extend = RobotCommands.extendClimb();
        retract = RobotCommands.retractClimb();
        stop = RobotCommands.stopClimb();


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

        commandSwerveDrivetrain.configureAutoBuilder();

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
            if (Robot.getAlliance().equals(DriverStation.Alliance.Red)) {
                if (robotCorner.getX() - Constants.RED_HUB_FRONT_CENTER.getX() > .05)
                    shotMode = ShotMode.SHOOTING;
            } else {
                if (Constants.BLUE_HUB_FRONT_CENTER.getX() - robotCorner.getX() > .05)
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

        calculateAutoClimbVelocities();

        // Logging
        logger.telemeterize(currentState);

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
        if (Robot.getAlliance().equals(DriverStation.Alliance.Blue)) return -requestYVelocity;
        return requestYVelocity;
    }

    public static double getRequestXVelocity() {
        if (Robot.getAlliance().equals(DriverStation.Alliance.Blue)) return -requestXVelocity;
        return requestXVelocity;
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


//        commandXboxController.y().onTrue(setHubSetpoint);
//        commandXboxController.x().onTrue(setLeftCornerSetpoint);
        //    commandXboxController.b().onTrue(setOutpostSetpoint);
//        commandXboxController.a().onTrue(setTowerSetpoint);
//
        commandXboxController.b().onTrue(new ParallelCommandGroup(new VelocityCommand(hopperSubsystem, 60), new VelocityCommand(indexerSubsystem, 30)))
                .onFalse(new ParallelCommandGroup(new VelocityCommand(hopperSubsystem, -5), new VelocityCommand(indexerSubsystem, IndexerConstants.IDLING_SPEED)));


//        commandXboxController.b().toggleOnTrue(new InstantCommand(() -> climbMode = climbMode.NOCLIMB))
//                .toggleOnFalse(new InstantCommand(() -> climbMode = climbMode.CLIMB));

//
//        commandXboxController.povLeft().onTrue(new InstantCommand(() -> climberSubsystem.setVelocity(-12)))
//                .onFalse(new InstantCommand(() -> climberSubsystem.setVoltage(0)));
//        commandXboxController.povRight().onTrue(new InstantCommand(() -> climberSubsystem.setVelocity(12)))
//                .onFalse(new InstantCommand(() -> climberSubsystem.setVoltage(0)));

        commandXboxController.povLeft().onTrue(Autos.driveToPose(ClimberSetpoint.CLIMB_LEFT_RED_PREP));
        commandXboxController.povRight().onTrue(Autos.pidAlign(ClimberSetpoint.CLIMB_LEFT_RED))
                .onFalse(commandSwerveDrivetrain.applyRequest(() -> drive
                        .withVelocityX(-requestXVelocity)
                        .withVelocityY(-requestYVelocity)
                        .withRotationalRate(requestRotationalVelocity)));

        commandXboxController.y().onTrue(new SequentialCommandGroup(
                Autos.driveToPose(ClimberSetpoint.CLIMB_LEFT_RED_PREP)
                        .alongWith(new PositionCommand(climberSubsystem, ClimberConstants.DEPLOY)),
                Autos.pidAlign(ClimberSetpoint.CLIMB_LEFT_RED).until(
                        () -> aligned() && (commandSwerveDrivetrain.getState().Speeds.vxMetersPerSecond < .01
                                && commandSwerveDrivetrain.getState().Speeds.vyMetersPerSecond < .01)),
                new PositionCommand(climberSubsystem, ClimberConstants.ZERO)
        ));

//        commandXboxController.povRight().onTrue(new PositionCommand(climberSubsystem, ClimberConstants.DEPLOY));
//        commandXboxController.povLeft().onTrue(new PositionCommand(climberSubsystem, ClimberConstants.CLIMB));
        commandXboxController.a().onTrue(new PositionCommand(climberSubsystem, ClimberConstants.ZERO));
        commandXboxController.b().onTrue(new PositionCommand(climberSubsystem, ClimberConstants.DEPLOY));

//        commandXboxController.povRight().onTrue(commandSwerveDrivetrain.applyRequest(() -> drive.withVelocityY(-1)));

    }


    public Command getAutonomousCommand() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            return Autos.autonChooserRed.getSelected();
        } else {
            return Autos.autonChooserBlue.getSelected();
        }
    }

    public static void calculateAutoClimbVelocities() {
        double currentX = getPose().getX();
        double currentY = getPose().getY();

        double currentRotation = getPose().getRotation().getDegrees();
        if (currentRotation < 0) currentRotation += 360;
        while (currentRotation >= 360) currentRotation -= 360;
        while (currentRotation <= -360) currentRotation += 360;

        double goalX = climberSetpoint.getPose2d().getX();
        double goalY = climberSetpoint.getPose2d().getY();

        if ((Math.abs(currentX - goalX) > .15
                || Math.abs(currentY - goalY) > .1)) {
            xVelocity = drivePIDControllerX.calculate(currentX, goalX);
            yVelocity = drivePIDControllerY.calculate(currentY, goalY);
        } else {
            xVelocity = drivePIDControllerXClose.calculate(currentX, goalX);
            yVelocity = drivePIDControllerYClose.calculate(currentY, goalY);
        }

        // Fixes backwards PID controller
        xVelocity = -xVelocity;
        yVelocity = -yVelocity;

        rotationVelocity = turnPIDController.calculate(currentRotation,
                climberSetpoint.getPose2d().getRotation().getDegrees());
        if (currentRotation > climberSetpoint.getPose2d().getRotation().getDegrees()) rotationVelocity -= .3;
        else rotationVelocity += .3;

//        System.out.println("X velocity: " + xVelocity);
//        System.out.println("Y velocity: " + yVelocity);
//        System.out.println("Rotational velocity: " + rotationVelocity);
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

    public boolean aligned() {
        return climberSetpoint.getPose2d().getTranslation().getDistance(getPose().getTranslation()) < .1;
    }
}
