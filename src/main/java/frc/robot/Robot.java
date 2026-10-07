// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.Vision.pendingMeasurements;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchState;
import org.wpilib.framework.TimedRobot;
import org.wpilib.system.Timer;

import com.pathplanner.lib.commands.PathfindingCommand;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeRollerSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.Vision;
import frc.robot.utility.ShotCalculator;

public class Robot extends TimedRobot {
    public static final HoodSubsystem hoodSubsystem = HoodSubsystem.getInstance();
    public static final HopperSubsystem hopperSubsystem = HopperSubsystem.getInstance();
    public static final IntakeRollerSubsystem intakeRollerSubsystem = IntakeRollerSubsystem.getInstance();
    public static final TurretSubsystem turretSubsystem = TurretSubsystem.getInstance();
    private final static Vision vision = Vision.getInstance();
    public static CommandSwerveDrivetrain commandSwerveDrivetrain = RobotContainer.commandSwerveDrivetrain;
    public static ShotCalculator shotCalculator = ShotCalculator.getInstance();
    public static Timer autonTimer = new Timer();
    private static Alliance alliance;
    // private static TalonFX motorLeader;
    // private static TalonFX motorFollower;
    private final RobotContainer m_robotContainer;
    private final List<Vision.VisionMeasurement> visionBatch = new ArrayList<>(3);
    // private final UserInterface userInterface = UserInterface.getInstance();
    private Command m_autonomousCommand;

    public Robot() {
        commandSwerveDrivetrain.configureAutoBuilder();
        m_robotContainer = new RobotContainer();

        //For sysid:
        // DataLogManager.start();
        // DriverStation.startDataLog(DataLogManager.getLog());

        CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand());

        // LimelightHelpers.setCameraPose_RobotSpace("limelight-left",
        //         -.316, -.316, .453, 0, 5, 135.218);
        // LimelightHelpers.setCameraPose_RobotSpace("limelight-right",
        //         -.317, .317, .436, 180, 5, -135.218);;

        addPeriodic(() -> {
                    shotCalculator.periodic();
                }, .020
        );
    }

    public static Alliance getAlliance() {
        return alliance;
    }

    public static double getAutoTime() {
        return autonTimer.get();
    }

    @Override
    public void robotPeriodic() {
        // System.out.println("Path: " + DataLogManager.getLogDir());

        // if (motorLeader != null) {
        //     if (motorLeader.isAlive()) {
        //         userInterface.setComponentData("Motor Percent (L)", motorLeader.get());
        //         userInterface.setComponentData("Motor Position (L)", motorLeader.getPosition().getValueAsDouble());
        //         userInterface.setComponentData("Motor Velocity (L)", motorLeader.getVelocity().getValueAsDouble());
        //         userInterface.setComponentData("Motor Voltage (L)", motorLeader.getMotorVoltage().getValueAsDouble());
        //     }
        // }

        visionBatch.clear();
        Vision.VisionMeasurement m;
        while ((m = pendingMeasurements.poll()) != null) {
            visionBatch.add(m);
        }
        visionBatch.sort(Comparator.comparingDouble(Vision.VisionMeasurement::timestampSeconds));

        for (Vision.VisionMeasurement measurement : visionBatch) {
            commandSwerveDrivetrain.addVisionMeasurement(measurement.pose(), measurement.timestampSeconds(), measurement.stdDevs());
        }
        RobotContainer.periodic();
        CommandScheduler.getInstance().run();

        RobotContainer.updateLastVelocity();
        // System.out.println("Pose Degrees: " + RobotContainer.getPose().getRotation().getDegrees());
        // System.out.println("Pigeon Degrees: " + RobotContainer.commandSwerveDrivetrain.getPigeon2().getYaw());
    }

    @Override
    public void disabledInit() {
        vision.getLeftLimelight().setThrottle(2000);
        vision.getFrontLimelight().setThrottle(2000);
        vision.getRightLimelight().setThrottle(2000);
    }

    @Override
    public void disabledPeriodic() {
        if (MatchState.getAlliance().isPresent()) alliance = MatchState.getAlliance().get();
    }

    @Override
    public void disabledExit() {
        vision.getLeftLimelight().setThrottle(0);
        vision.getFrontLimelight().setThrottle(0);
        vision.getRightLimelight().setThrottle(0);
    }

    @Override
    public void autonomousInit() {
        if (getAlliance() != null && getAlliance().equals(Alliance.RED)) {
            commandSwerveDrivetrain.getPigeon2().setYaw(180d);
        } else {
            commandSwerveDrivetrain.getPigeon2().setYaw(0);
        }

        commandSwerveDrivetrain.seedFieldCentric();

        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }

        autonTimer.restart();
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
    }

    @Override
    public void teleopInit() {
        CommandScheduler.getInstance().cancelAll();
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }

        CommandScheduler.getInstance().schedule(RobotCommands.idleState());
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void teleopExit() {
    }

    @Override
    public void utilityInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void utilityPeriodic() {
    }

    @Override
    public void utilityExit() {
    }

    @Override
    public void simulationPeriodic() {
    }
}
