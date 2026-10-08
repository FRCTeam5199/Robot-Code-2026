package frc.robot.autos;

import frc.robot.Autos;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.opmode.Autonomous;
import org.wpilib.opmode.PeriodicOpMode;

@Autonomous(name = "Right Double Trench")
public class RightDoubleTrench extends PeriodicOpMode {
    private static final Autos autos = Autos.getInstance();

    @Override
    public void start() {
        CommandScheduler.getInstance().schedule(autos.getRightDoubleTrench());
    }
}