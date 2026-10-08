package frc.robot.autos;

import frc.robot.Autos;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.opmode.Autonomous;
import org.wpilib.opmode.PeriodicOpMode;

@Autonomous(name = "Right Delayed Trench")
public class RightDelayedTrench extends PeriodicOpMode {
    private static final Autos autos = Autos.getInstance();

    @Override
    public void start() {
        CommandScheduler.getInstance().schedule(autos.getRightDelayedTrench());
    }
}