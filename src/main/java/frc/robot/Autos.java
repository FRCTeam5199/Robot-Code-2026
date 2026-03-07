package frc.robot;

/// / Copyright (c) FIRST and other WPILib contributors.
/// / Open Source Software; you can modify and/or share it under the terms of
/// / the WPILib BSD license file in the root directory of this project.

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.constants.Constants;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.*;
import frc.robot.utility.ClimberSetpoint;
import frc.robot.utility.ShotCalculator;

public final class Autos {
    private SendableChooser<Command> autoChooser;
    private static Autos autos;

    public static SendableChooser<Command> autonChooserRed = new SendableChooser<>();
    public static SendableChooser<Command> autonChooserBlue = new SendableChooser<>();

    private static PathPlannerAuto redBottomScore;
    private static PathPlannerAuto redTopScore;

    private static PathPlannerAuto redBottomShuttle;
    private static PathPlannerAuto redTopShuttle;

    private static PathPlannerAuto blueBottomScore;
    private static PathPlannerAuto blueTopScore;

    private static PathPlannerAuto blueBottomShuttle;
    private static PathPlannerAuto blueTopShuttle;

    private static Command redBottomStartToMid;
    // Bottom Shuttling Paths- Red
    private static Command redBottomMidFullClear;
    private static Command redTopStartToClimb;
    // Bottom Shooting Paths - Red
    private static Command redBottomMidHalfClear;
    private static Command redBottomMidToScore;

    private static Command blueBottomStartToMid;
    // Bottom Shuttling Paths- Blue
    private static Command blueBottomMidFullClear;
    private static Command blueTopStartToClimb;
    // Bottom Shooting Paths - Blue
    private static Command blueBottomMidHalfClear;
    private static Command blueBottomMidToScore;
    //------------------------------------------------------------------------
    private static Command redTopStartToMid;
    // Top Shuttling Paths- Red
    private static Command redTopMidFullClear;
    private static Command redBottomStartToClimb;
    // Top Shooting Paths - Red
    private static Command redTopMidHalfClear;
    private static Command redTopMidToScore;

    private static Command blueTopStartToMid;
    // Bottom Shooting Paths- Blue
    private static Command blueTopMidFullClear;
    private static Command blueBottomStartToClimb;
    // Bottom Shuttling Paths - Blue
    private static Command blueTopMidHalfClear;
    private static Command blueTopMidToScore;

    public static final CommandSwerveDrivetrain commandSwerveDrivetrain = RobotContainer.commandSwerveDrivetrain;
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

    public static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric().withDesaturateWheelSpeeds(true)
            .withDeadband(Constants.MAX_SPEED * .05).withRotationalDeadband(Constants.MAX_ANGULAR_RATE * .05) // Add a 10% deadband
            .withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.OpenLoopVoltage);


    /**
     * Gets or creates the AutoChooser (Singleton Method)
     */
    public SendableChooser<Command> getAutoChooser() {
        if (autoChooser == null) {
            autoChooser = AutoBuilder.buildAutoChooser();
//            UserInterface.getTab("Auton").add("AutoChooser", autoChooser).withWidget(BuiltInWidgets.kComboBoxChooser).withSize(1, 1).withPosition(0, 0);
        }

        return autoChooser;
    }

    public static void initializeAutos() {


//        fourPieceBlueBottomL4 = new PathPlannerAuto("4 Piece Blue Bottom L4");


        Shuffleboard.getTab("Autons").add("Red Autons", autonChooserRed)
                .withWidget(BuiltInWidgets.kComboBoxChooser).withPosition(0, 0)
                .withSize(2, 1);
        Shuffleboard.getTab("Autons").add("Blue Autons", autonChooserBlue)
                .withWidget(BuiltInWidgets.kComboBoxChooser).withPosition(0, 0)
                .withSize(2, 1);


        //autonChooserRed.addOption();


//        autonChooserBlue.addOption();

    }

    public static Command driveToPose(double goalX, double goalY, double goalDegrees) {
        return new SequentialCommandGroup(
                new InstantCommand(() ->
                        commandSwerveDrivetrain.applyRequest(() ->
                                drive.withVelocityX(0).
                                        withVelocityX(0).
                                        withRotationalRate(0)
                        )
                ),
                AutoBuilder.pathfindToPose(
                        new Pose2d(goalX, goalY, Rotation2d.fromDegrees(goalDegrees)),
                        new PathConstraints(2d, 2d, Units.degreesToRadians(540d), Units.degreesToRadians(720d)),
                        0d
                )
        );
    }

    public static Command driveToPose(ClimberSetpoint climberSetpoint) {
        return new SequentialCommandGroup(
                new InstantCommand(() ->
                        commandSwerveDrivetrain.applyRequest(() ->
                                drive.withVelocityX(0).
                                        withVelocityX(0).
                                        withRotationalRate(0)
                        )
                ),

                AutoBuilder.pathfindToPose(
                        climberSetpoint.getPose2d(),
                        new PathConstraints(1, 3,
                                Units.degreesToRadians(540d), Units.degreesToRadians(720d)), 1d));

    }

    public static Command pidAlign(ClimberSetpoint climberSetpoint) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> RobotContainer.setClimberSetpoint(climberSetpoint)),
                new FunctionalCommand(
                        () -> {
                            commandSwerveDrivetrain.setControl(
                                    drive.withVelocityX(0)
                                            .withVelocityY(0)
                                            .withRotationalRate(0));
                        },
                        () -> {
                            commandSwerveDrivetrain.setControl(
                                    drive.withVelocityX(RobotContainer.getXVelocity())
                                            .withVelocityY(RobotContainer.getYVelocity())
                                            .withRotationalRate(RobotContainer.getRotationVelocity()));
                        },
                        (interrupted) -> {
                            commandSwerveDrivetrain.setControl(
                                    drive.withVelocityX(0)
                                            .withVelocityY(0)
                                            .withRotationalRate(0));
                        },
                        () -> false/* (commandSwerveDrivetrain.getState().Speeds.vxMetersPerSecond < .01
                                && commandSwerveDrivetrain.getState().Speeds.vyMetersPerSecond < .01)*/,
                        commandSwerveDrivetrain
                )
        );
    }

    public static Command driveToPose(ClimberSetpoint climberSetpoint,
                                      double maxVelocity, double maxAcceleration, double goalEndVelocity) {
        return new SequentialCommandGroup(
                new InstantCommand(() ->
                        commandSwerveDrivetrain.applyRequest(() ->
                                drive.withVelocityX(0).
                                        withVelocityX(0).
                                        withRotationalRate(0)
                        )
                ),
                AutoBuilder.pathfindToPose(
                        climberSetpoint.getPose2d(),
                        new PathConstraints(maxVelocity, maxAcceleration,
                                Units.degreesToRadians(540d), Units.degreesToRadians(720d)), goalEndVelocity)
        );
    }

    public static Autos getInstance() {
        if (autos == null) {
            autos = new Autos();
        }
        return autos;
    }
}


