package frc.robot;

/// / Copyright (c) FIRST and other WPILib contributors.
/// / Open Source Software; you can modify and/or share it under the terms of
/// / the WPILib BSD license file in the root directory of this project.

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

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
    private static PathPlannerAuto redBottomDelayedBump;
    private static PathPlannerAuto redTopDelayedBump;
    private static PathPlannerAuto redBottomDoubleTrench;
    private static PathPlannerAuto redTopDoubleBump;
    private static PathPlannerAuto redTopDoubleTrench;
    private static PathPlannerAuto redBottomDoubleBump;
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

        redBottomDoubleTrench = new PathPlannerAuto("Red Bottom Double Trench");
        redTopDoubleTrench = new PathPlannerAuto("Red Top Double Trench");

        redBottomDoubleBump = new PathPlannerAuto("Red Bottom Double Bump");
        redTopDoubleBump = new PathPlannerAuto("Red Top Double Bump");

        redBottomDelayedBump = new PathPlannerAuto("Red Bottom Delayed Bump");
        redTopDelayedBump = new PathPlannerAuto("Red Top Delayed Bump");

        Shuffleboard.getTab("Autons").add("Red Autons", autonChooserRed)
                .withWidget(BuiltInWidgets.kComboBoxChooser).withPosition(0, 0)
                .withSize(2, 1);
        Shuffleboard.getTab("Autons").add("Blue Autons", autonChooserBlue)
                .withWidget(BuiltInWidgets.kComboBoxChooser).withPosition(2, 0)
                .withSize(2, 1);

        autonChooserRed.addOption("Red Left Double Trench", redBottomDoubleTrench);
        autonChooserRed.addOption("Red Right Double Trench", redTopDoubleTrench);
        autonChooserRed.addOption("Red Left Double Bump", redBottomDoubleBump);
        autonChooserRed.addOption("Red Right Double Bump", redTopDoubleBump);
        autonChooserRed.addOption("Red Left Delayed Bump", redBottomDelayedBump);
        autonChooserRed.addOption("Red Right Delayed Bump", redTopDelayedBump);

        autonChooserBlue.addOption("Blue Left Double Trench", redBottomDoubleTrench);
        autonChooserBlue.addOption("Blue Right Double Trench", redTopDoubleTrench);
        autonChooserRed.addOption("Blue Left Double Bump", redBottomDoubleBump);
        autonChooserRed.addOption("Blue Right Double Bump", redTopDoubleBump);
        autonChooserRed.addOption("Blue Left Delayed Bump", redBottomDelayedBump);
        autonChooserRed.addOption("Blue Right Delayed Bump", redTopDelayedBump);

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


