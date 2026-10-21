package frc.robot;

import org.wpilib.command2.Command;
import org.wpilib.tunable.Selectable;
import org.wpilib.tunable.Tunables;

import com.pathplanner.lib.commands.PathPlannerAuto;

public final class Autos {
    // private static NetworkTableInstance inst = NetworkTableInstance.getDefault();
    // private static NetworkTable table = inst.getTable("Autons");
    private static Selectable<Command> autoChooserRed = new Selectable<Command>();
    private static Selectable<Command> autoChooserBlue = new Selectable<Command>();

    private static Autos autos;

    public void initializeAutos() {
        // Red
        autoChooserRed.addDefault("NONE", new PathPlannerAuto("Red Bottom Double Trench"));

        autoChooserRed.add("Red Bottom Double Trench", new PathPlannerAuto("Red Bottom Double Trench"));
        autoChooserRed.add("Red Top Double Trench", new PathPlannerAuto("Red Top Double Trench"));
        
        autoChooserRed.add("Red Bottom Delayed Trench", new PathPlannerAuto("Red Bottom Delayed Trench"));
        autoChooserRed.add("Red Top Delayed Trench", new PathPlannerAuto("Red Top Delayed Trench"));
        
        autoChooserRed.add("Red Bottom Double Bump", new PathPlannerAuto("Red Bottom Double Bump"));
        autoChooserRed.add("Red Top Double Bump", new PathPlannerAuto("Red Top Double Bump"));

        autoChooserRed.add("Red Bottom Delayed Bump", new PathPlannerAuto("Red Bottom Delayed Bump"));
        autoChooserRed.add("Red Top Delayed Bump", new PathPlannerAuto("Red Top Delayed Bump"));
        
        // Blue
        // autoChooserBlue.addDefault("NONE", new PathPlannerAuto("Blue Bottom Double Trench"));

        // autoChooserBlue.add("Blue Bottom Double Trench", new PathPlannerAuto("Blue Bottom Double Trench"));
        // autoChooserBlue.add("Blue Top Double Trench", new PathPlannerAuto("Blue Top Double Trench"));
        
        // autoChooserBlue.add("Blue Bottom Delayed Trench", new PathPlannerAuto("Blue Bottom Delayed Trench"));
        // autoChooserBlue.add("Blue Top Delayed Trench", new PathPlannerAuto("Blue Top Delayed Trench"));
        
        // autoChooserBlue.add("Blue Bottom Double Bump", new PathPlannerAuto("Blue Bottom Double Bump"));
        // autoChooserBlue.add("Blue Top Double Bump", new PathPlannerAuto("Blue Top Double Bump"));

        // autoChooserBlue.add("Blue Bottom Delayed Bump", new PathPlannerAuto("Blue Bottom Delayed Bump"));
        // autoChooserBlue.add("Blue Top Delayed Bump", new PathPlannerAuto("Blue Top Delayed Bump"));
        
        if (!Tunables.publish("Auto Chooser", autoChooserRed)) System.out.println("ELASTIC AUTOCHOOSER RED INIT FAILED");
        // if (!Tunables.publish("Auto Chooser Blue", autoChooserBlue)) System.out.println("ELASTIC AUTOCHOOSER BLUE INIT FAILED");
    }

    public static Autos getInstance() {
        if (autos == null) autos = new Autos();
        return autos;
    }

    public Command getRedSelected() {
        return autoChooserRed.getSelected();
    }

    public Command getBlueSelected() {
        return autoChooserBlue.getSelected();
    }
}
