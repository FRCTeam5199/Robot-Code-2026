package frc.robot;

import org.wpilib.command2.CommandScheduler;
import org.wpilib.opmode.PeriodicOpMode;
import org.wpilib.opmode.Teleop;

@Teleop
public class Teleoperated extends PeriodicOpMode {
    @Override
    public void start() {
        CommandScheduler.getInstance().schedule(RobotCommands.idleState());
    }
}
