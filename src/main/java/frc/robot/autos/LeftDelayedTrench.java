package frc.robot;

import org.wpilib.command2.CommandScheduler;
import org.wpilib.opmode.Autonomous;
import org.wpilib.opmode.PeriodicOpMode;

@Autonomous(name = "Left Delayed Trench")
public class LeftDelayedTrench extends PeriodicOpMode {
    private static final Autos autos = Autos.getInstance();

    @Override
    public void start() {
        CommandScheduler.getInstance().schedule(autos.getLeftDelayedTrench());
    }
}