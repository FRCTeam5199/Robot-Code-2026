package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;
import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.opmode.Autonomous;
import org.wpilib.opmode.PeriodicOpMode;

@Autonomous(name = "Red Left Double Trench")
public class Test extends PeriodicOpMode {
    PathPlannerAuto auto = new PathPlannerAuto("Test");

    @Override
    public void start() {
        CommandScheduler.getInstance().schedule(auto);
    }
}
