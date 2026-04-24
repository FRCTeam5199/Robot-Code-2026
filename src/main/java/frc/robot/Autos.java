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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.constants.Constants;
import frc.robot.subsystems.*;
import frc.robot.utility.ShotCalculator;

public final class Autos {
    public static final CommandSwerveDrivetrain commandSwerveDrivetrain = RobotContainer.commandSwerveDrivetrain;
    public static final IntakeRollerSubsystem intakeRollerSubsystem = IntakeRollerSubsystem.getInstance();
    public static final IntakePivotSubsystem intakePivotSubsystem = IntakePivotSubsystem.getInstance();
    public static final HopperSubsystem hopperSubsystem = HopperSubsystem.getInstance();
    public static final ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();
    public static final KickerSubsystem kickerSubsystem = KickerSubsystem.getInstance();
    public static final HoodSubsystem hoodSubsystem = HoodSubsystem.getInstance();

    //    private static PathPlannerAuto redBottomShuttle;
//    private static PathPlannerAuto redTopShuttle;
    public static final ShotCalculator shotCalculator = ShotCalculator.getInstance();
    public static final TurretSubsystem turretSubsystem = TurretSubsystem.getInstance();
//    public static final ClimberSubsystem climberSubsystem = ClimberSubsystem.getInstance();

    //    private static PathPlannerAuto blueBottomShuttle;
//    private static PathPlannerAuto blueTopShuttle;
    public static final Vision vision = Vision.getInstance();
    public static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric().withDesaturateWheelSpeeds(true)
            .withDeadband(Constants.MAX_SPEED * .05).withRotationalDeadband(Constants.MAX_ANGULAR_RATE * .05) // Add a 10% deadband
            .withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.OpenLoopVoltage);
    public static SendableChooser<Command> autonChooserRed = new SendableChooser<>();
    public static SendableChooser<Command> autonChooserBlue = new SendableChooser<>();
    private static Autos autos;
    private static PathPlannerAuto redBottomScore;
    private static PathPlannerAuto redTopScore;
    private static PathPlannerAuto redBottomTrenchDelayedScore;
    private static PathPlannerAuto redTopTrenchDelayedScore;
    private static PathPlannerAuto redBottomDoubleScore;
    private static PathPlannerAuto redTopBumpDoubleScore;
    private static PathPlannerAuto redTopDoubleScore;
    private static PathPlannerAuto redBottomBumpDoubleScore;
    private static PathPlannerAuto redBottomSelfShuttle;
    private static PathPlannerAuto redTopSelfShuttle;
    private static PathPlannerAuto blueBottomScore;
    private static PathPlannerAuto blueTopScore;
    private static SequentialCommandGroup blueBottomSelfShuttle;
    private static SequentialCommandGroup blueTopSelfShuttle;
    private static PathPlannerAuto blueBottomDoubleScore;
    private static PathPlannerAuto blueTopDoubleScore;
    private static Timer pidAlignmentTimer = new Timer();
    private SendableChooser<Command> autoChooser;

    public static void initializeAutos() {

        redBottomScore = new PathPlannerAuto("Red Bottom Score");
        redTopScore = new PathPlannerAuto("Red Top Score");

        redBottomDoubleScore = new PathPlannerAuto("Red Bottom Double Score");
        redTopDoubleScore = new PathPlannerAuto("Red Top Double Score");
        redBottomBumpDoubleScore = new PathPlannerAuto("Red Bottom Bump Double Score Plus Half");
        redTopBumpDoubleScore = new PathPlannerAuto("Red Top Bump Double Score Plus Half");

        redBottomTrenchDelayedScore = new PathPlannerAuto("Red Bottom Trench Delayed Plus Half");
        redTopTrenchDelayedScore = new PathPlannerAuto("Red Top Trench Delayed Plus Half");

        redBottomSelfShuttle = new PathPlannerAuto("Red Bottom Self Shuttle");
        redTopSelfShuttle = new PathPlannerAuto("Red Top Self Shuttle");

        blueBottomSelfShuttle = new PathPlannerAuto("Copy of Red Bottom Self Shuttle").andThen(RobotCommands.indexBallsAuto());
        blueTopSelfShuttle = new PathPlannerAuto("Copy of Red Top Self Shuttle").andThen(RobotCommands.indexBallsAuto());

        blueBottomDoubleScore = new PathPlannerAuto("Copy of Red Bottom Double Score");
        blueTopDoubleScore = new PathPlannerAuto("Copy of Red Top Double Score");

        blueBottomScore = new PathPlannerAuto("Copy of Red Bottom Score");
        blueTopScore = new PathPlannerAuto("Copy of Red Top Score");

        Shuffleboard.getTab("Autons").add("Red Autons", autonChooserRed)
                .withWidget(BuiltInWidgets.kComboBoxChooser).withPosition(0, 0)
                .withSize(2, 1);
        Shuffleboard.getTab("Autons").add("Blue Autons", autonChooserBlue)
                .withWidget(BuiltInWidgets.kComboBoxChooser).withPosition(2, 0)
                .withSize(2, 1);


        // autonChooserRed.addOption("Red Left Score Climb", redBottomScore);
        // autonChooserRed.addOption("Red Right Score Climb", redTopScore);

//        autonChooserRed.addOption("Red Left Self Shuttle", redBottomSelfShuttle);
//        autonChooserRed.addOption("Red Right Self Shuttle", redTopSelfShuttle);

        autonChooserRed.addOption("Red Left Double Score", redBottomDoubleScore);
        autonChooserRed.addOption("Red Right Double Score", redTopDoubleScore);
        autonChooserRed.addOption("Red Left Double Bump Score", redBottomBumpDoubleScore);
        autonChooserRed.addOption("Red Right Double Bump Score", redTopBumpDoubleScore);
        autonChooserRed.addOption("Red Left Single Delayed Score", redBottomTrenchDelayedScore);
        autonChooserRed.addOption("Red Right Single Delayed Score", redTopTrenchDelayedScore);

//        autonChooserBlue.addOption("Blue Left Self Shuttle", blueBottomSelfShuttle);
//        autonChooserBlue.addOption("Blue Right Self Shuttle", blueTopSelfShuttle);

        autonChooserBlue.addOption("Blue Left Double Score", blueBottomDoubleScore);
        autonChooserBlue.addOption("Blue Right Double Score", blueTopDoubleScore);

        // autonChooserBlue.addOption("Blue Left Score Climb", blueBottomScore);
        // autonChooserBlue.addOption("Blue Right Score Climb", blueTopScore);

    }

    public static Autos getInstance() {
        if (autos == null) {
            autos = new Autos();
        }
        return autos;
    }

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


}


