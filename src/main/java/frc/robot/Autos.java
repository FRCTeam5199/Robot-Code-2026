package frc.robot;

import org.wpilib.command2.Command;
import org.wpilib.command2.PrintCommand;
import org.wpilib.tunable.Selectable;
import org.wpilib.tunable.Tunables;

import com.pathplanner.lib.commands.PathPlannerAuto;

public final class Autos {
    private static Selectable<Command> autoChooser = new Selectable<Command>();

    private static Autos autos;

    public static Autos getInstance() {
        if (autos == null) autos = new Autos();
        return autos;
    }

    public void initializeAutos() {
        // Red
        autoChooser.addDefault("NONE", new PrintCommand("No Auto Selected"));

        autoChooser.add("Left Double Trench Slow", new PathPlannerAuto("Red Bottom Double Trench Slow"));
        autoChooser.add("Right Double Trench Slow", new PathPlannerAuto("Red Top Double Trench Slow"));

        autoChooser.add("Left Delayed Trench", new PathPlannerAuto("Red Bottom Delayed Trench"));
        autoChooser.add("Right Delayed Trench", new PathPlannerAuto("Red Top Delayed Trench"));

        autoChooser.add("Left Delayed Bump", new PathPlannerAuto("Red Bottom Delayed Bump"));
        autoChooser.add("Right Delayed Bump", new PathPlannerAuto("Red Top Delayed Bump"));

        if (!Tunables.publish("Auto Chooser", autoChooser)) System.out.println("ELASTIC AUTOCHOOSER INIT FAILED");
    }

    public Command getSelected() {
        return autoChooser.getSelected();
    }
}
