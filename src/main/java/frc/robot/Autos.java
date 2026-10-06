package frc.robot;

import java.util.Collection;
import java.util.List;

import org.wpilib.command2.Command;
import org.wpilib.command2.SequentialCommandGroup;
import org.wpilib.tunable.Selectable;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.commands.PathPlannerAuto;

import frc.robot.constants.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakePivotSubsystem;
import frc.robot.subsystems.IntakeRollerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.Vision;
import frc.robot.utility.ShotCalculator;

public final class Autos {
    public static final CommandSwerveDrivetrain commandSwerveDrivetrain = RobotContainer.commandSwerveDrivetrain;
    public static final IntakeRollerSubsystem intakeRollerSubsystem = IntakeRollerSubsystem.getInstance();
    public static final IntakePivotSubsystem intakePivotSubsystem = IntakePivotSubsystem.getInstance();
    public static final HopperSubsystem hopperSubsystem = HopperSubsystem.getInstance();
    public static final ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();
    public static final IndexerSubsystem indexerSubsystem = IndexerSubsystem.getInstance();
    public static final HoodSubsystem hoodSubsystem = HoodSubsystem.getInstance();
    public static final ShotCalculator shotCalculator = ShotCalculator.getInstance();
    public static final TurretSubsystem turretSubsystem = TurretSubsystem.getInstance();
    public static final Vision vision = Vision.getInstance();
    public static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric().withDesaturateWheelVelocities(true)
            .withDeadband(Constants.MAX_SPEED * .05).withRotationalDeadband(Constants.MAX_ANGULAR_RATE * .05) // Add a 10% deadband
            .withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.OpenLoopVoltage);
    private static Selectable<Command> autoChooserRed = new Selectable<Command>();
    private static Selectable<Command> autoChooserBlue = new Selectable<Command>();
    private static PathPlannerAuto redBottomDelayedBump;
    private static PathPlannerAuto redTopDelayedBump;
    private static PathPlannerAuto redBottomDoubleTrench;
    private static PathPlannerAuto redTopDoubleBump;
    private static PathPlannerAuto redTopDoubleTrench;
    private static PathPlannerAuto redBottomDoubleBump;
    private static PathPlannerAuto redBottomDelayedTrench;
    private static PathPlannerAuto redTopDelayedTrench;
    private static PathPlannerAuto blueBottomDelayedBump;
    private static PathPlannerAuto blueTopDelayedBump;
    private static PathPlannerAuto blueBottomDoubleTrench;
    private static PathPlannerAuto blueTopDoubleBump;
    private static PathPlannerAuto blueTopDoubleTrench;
    private static PathPlannerAuto blueBottomDoubleBump;
    private static PathPlannerAuto blueBottomDelayedTrench;
    private static PathPlannerAuto blueTopDelayedTrench;
    private static Autos autos;

    public static void initializeAutos() {
        autoChooserRed.addDefault("Red Left Double Trench", redBottomDoubleTrench);
        autoChooserRed.add("Red Right Double Trench", redTopDoubleTrench);
        autoChooserRed.add("Red Left Double Bump", redBottomDoubleBump);
        autoChooserRed.add("Red Right Double Bump", redTopDoubleBump);
        autoChooserRed.add("Red Left Delayed Bump", redBottomDelayedBump);
        autoChooserRed.add("Red Right Delayed Bump", redTopDelayedBump);
        autoChooserRed.add("Red Left Delayed Trench", redBottomDelayedTrench);
        autoChooserRed.add("Red Right Delayed Trench", redTopDelayedTrench);

        autoChooserBlue.add("Blue Left Double Trench", blueBottomDoubleTrench);
        autoChooserBlue.add("Blue Right Double Trench", blueTopDoubleTrench);
        autoChooserBlue.add("Blue Left Double Bump", blueBottomDoubleBump);
        autoChooserBlue.add("Blue Right Double Bump", blueTopDoubleBump);
        autoChooserBlue.add("Blue Left Delayed Bump", blueBottomDelayedBump);
        autoChooserBlue.add("Blue Right Delayed Bump", blueTopDelayedBump);
        autoChooserBlue.add("Blue Left Delayed Trench", blueBottomDelayedTrench);
        autoChooserBlue.add("Blue Right Delayed Trench", blueTopDelayedTrench);
    }

    public static Autos getInstance() {
        if (autos == null) autos = new Autos();
        return autos;
    }

   public Selectable<Command> getAutoChooser() {
       return autoChooserRed;
   }
}
