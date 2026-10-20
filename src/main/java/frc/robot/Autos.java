package frc.robot;

import org.wpilib.command2.Command;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.tunable.Selectable;

import com.pathplanner.lib.commands.PathPlannerAuto;

public final class Autos {
    private static NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private static NetworkTable table = inst.getTable("Autons");
    private static Selectable<Command> autoChooserRed = new Selectable<Command>();
    private static Selectable<Command> autoChooserBlue = new Selectable<Command>();

    private static Autos autos;

    public void initializeAutos() {
        autoChooserRed.addDefault("Red Bottom Double Trench", new PathPlannerAuto("Red Bottom Double Trench"));
        autoChooserRed.add("Red Top Double Trench", new PathPlannerAuto("Red Top Double Trench"));
        
        autoChooserRed.add("Red Bottom Delayed Trench", new PathPlannerAuto("Red Bottom Delayed Trench"));
        autoChooserRed.add("Red Top Delayed Trench", new PathPlannerAuto("Red Top Delayed Trench"));
        
        autoChooserRed.add("Red Bottom Double Bump", new PathPlannerAuto("Red Bottom Double Bump"));
        autoChooserRed.add("Red Top Double Bump", new PathPlannerAuto("Red Top Double Bump"));

        autoChooserRed.add("Red Bottom Delayed Bump", new PathPlannerAuto("Red Bottom Delayed Bump"));
        autoChooserRed.add("Red Top Delayed Bump", new PathPlannerAuto("Red Top Delayed Bump"));
        
        table.getEntry("Auto Chooser Red");
    }

    public static Autos getInstance() {
        if (autos == null) autos = new Autos();
        return autos;
    }

    public Command getSelected() {
        return autoChooserRed.getSelected();
    }
}
