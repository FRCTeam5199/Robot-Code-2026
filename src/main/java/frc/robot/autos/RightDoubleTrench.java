package frc.robot.autos;

import frc.robot.Autos;
import frc.robot.Robot;
import frc.robot.RobotCommands;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.driverstation.Alliance;
import org.wpilib.math.geometry.Rotation2d;
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