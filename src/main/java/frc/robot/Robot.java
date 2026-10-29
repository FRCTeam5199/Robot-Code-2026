// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.pathplanner.lib.commands.PathfindingCommand;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.KickerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeRollerSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.templates.VelocityCommand;

public class Robot extends TimedRobot {
    private final RobotContainer m_robotContainer;
    public static final KickerSubsystem kickerSubsystem = KickerSubsystem.getInstance();
    public static final IntakeRollerSubsystem intakeRollerSubsystem = IntakeRollerSubsystem.getInstance();
    public static CommandSwerveDrivetrain commandSwerveDrivetrain = RobotContainer.commandSwerveDrivetrain;
    private Command m_autonomousCommand;
    private static DriverStation.Alliance alliance;

    public Robot() {   
        commandSwerveDrivetrain.configureAutoBuilder();
        m_robotContainer = new RobotContainer();
        Logger.addDataReceiver(new WPILOGWriter());
    }

    @Override
    public void robotInit() {
        DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog());
        CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand());
    }

    @Override
    public void robotPeriodic() {
        RobotContainer.periodic();
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {
        if (DriverStation.getAlliance().isPresent()) alliance = DriverStation.getAlliance().get();
    }

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        if (getAlliance() != null && getAlliance().equals(DriverStation.Alliance.Red)) {
            commandSwerveDrivetrain.getPigeon2().setYaw(180d);
        } else {
            commandSwerveDrivetrain.getPigeon2().setYaw(0);
        }
        commandSwerveDrivetrain.seedFieldCentric();

        m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {
    }

    @Override
    public void teleopInit() {
        RobotContainer.speedMultiplier = 1.0;
        CommandScheduler.getInstance().cancelAll();
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
        CommandScheduler.getInstance().schedule(new VelocityCommand(kickerSubsystem, KickerConstants.KICKER_REVERSE));
    }

    @Override
    public void teleopPeriodic() {

    }

    @Override
    public void teleopExit() {
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void testExit() {
    }

    @Override
    public void simulationPeriodic() {
    }

    public static DriverStation.Alliance getAlliance() {
        return alliance;
    }
}
