package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.IntakeRollerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.*;

public class RobotCommands {
    public static final IntakeRollerSubsystem intakeRollerSubsystem = IntakeRollerSubsystem.getInstance();
    private static final KickerSubsystem kickerSubsystem = KickerSubsystem.getInstance();
    private static final HoodSubsystem hoodSubsystem = HoodSubsystem.getInstance();
    private static final ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();
    private static boolean hasStartedTimer;
    private static Timer timer = new Timer();

    
}