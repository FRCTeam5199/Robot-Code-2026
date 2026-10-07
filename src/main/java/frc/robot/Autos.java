package frc.robot;

import frc.robot.utility.Elastic;
import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.opmode.Autonomous;
import org.wpilib.opmode.PeriodicOpMode;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.telemetry.TelemetryRegistry;
import org.wpilib.tunable.Selectable;

import com.pathplanner.lib.commands.PathPlannerAuto;
import org.wpilib.tunable.Tunables;

public final class Autos {
    private static Selectable<Command> autoChooser = new Selectable<>();
    private static PathPlannerAuto redBottomDelayedBump;
    private static PathPlannerAuto redTopDelayedBump;
    private static PathPlannerAuto redBottomDoubleTrench;
    private static PathPlannerAuto redTopDoubleTrench;
    private static PathPlannerAuto redBottomDelayedTrench;
    private static PathPlannerAuto redTopDelayedTrench;
    private static Autos autos;

    public static void initializeAutos() {
        redBottomDoubleTrench = new PathPlannerAuto("Red Bottom Double Trench");
        redTopDoubleTrench = new PathPlannerAuto("Red Top Double Trench");

        redBottomDelayedBump = new PathPlannerAuto("Red Bottom Delayed Bump");
        redTopDelayedBump = new PathPlannerAuto("Red Top Delayed Bump");

        redBottomDelayedTrench = new PathPlannerAuto("Red Bottom Delayed Trench");
        redTopDelayedTrench = new PathPlannerAuto("Red Top Delayed Trench");

        autoChooser.add("Red Left Double Trench", redBottomDoubleTrench);
        autoChooser.add("Red Right Double Trench", redTopDoubleTrench);

        autoChooser.add("Red Left Delayed Bump", redBottomDelayedBump);
        autoChooser.add("Red Right Delayed Bump", redTopDelayedBump);

        autoChooser.add("Red Left Delayed Trench", redBottomDelayedTrench);
        autoChooser.add("Red Right Delayed Trench", redTopDelayedTrench);
    }

    public static Autos getInstance() {
        if (autos == null) autos = new Autos();
        return autos;
    }

    public Selectable<Command> getAutoChooser() {
        return autoChooser;
    }

    @Autonomous(name = "Red Left Double Trench")
    class LeftDoubleTrench extends PeriodicOpMode {

        @Override
        public void start() {
            CommandScheduler.getInstance().schedule(redBottomDoubleTrench);
        }
    }
}
