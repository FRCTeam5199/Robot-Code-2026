// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.HopperConstants;
import frc.robot.constants.IndexerConstants;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeRollerSubsystem;
import frc.robot.subsystems.TurretSubsystem;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeRollerSubsystem;
import frc.robot.subsystems.templates.VelocityCommand;
import frc.robot.utility.LimelightHelpers;

public class Robot extends TimedRobot {
    //    public static final HoodSubsystem hoodSubsystem = HoodSubsystem.getInstance();
    public static final IndexerSubsystem indexerSubsystem = IndexerSubsystem.getInstance();
    public static final HopperSubsystem hopperSubsystem = HopperSubsystem.getInstance();
    public static final IntakeRollerSubsystem intakerollersubsystem = IntakeRollerSubsystem.getInstance();
    public static final TurretSubsystem turretSubsystem = TurretSubsystem.getInstance();
    public static CommandSwerveDrivetrain commandSwerveDrivetrain = RobotContainer.commandSwerveDrivetrain;
    public static Timer originTimer = new Timer();
    // private static TalonFX motorLeader;
    // private static TalonFX motorFollower;
    private final RobotContainer m_robotContainer;
    // private final UserInterface userInterface = UserInterface.getInstance();
    private Command m_autonomousCommand;


    public Robot() {
        m_robotContainer = new RobotContainer();

        // userInterface.createComponent("Motor ID (L)", "Control", BuiltInWidgets.kTextView, 0, 0, 1, 1, 0);
        // userInterface.createComponent("Set (L)", "Control", BuiltInWidgets.kToggleButton, 0, 1, 1, 1, false);

        // userInterface.createComponent("Motor Percent (L)", "Control", BuiltInWidgets.kTextView, 1, 0, 1, 1, 0);
        // userInterface.createComponent("Motor Position (L)", "Control", BuiltInWidgets.kTextView, 1, 1, 1, 1, 0);
        // userInterface.createComponent("Motor Velocity (L)", "Control", BuiltInWidgets.kTextView, 1, 2, 1, 1, 0);
        // userInterface.createComponent("Motor Voltage (L)", "Control", BuiltInWidgets.kTextView, 1, 3, 1, 1, 0);

        // userInterface.createComponent("Set Percent (L)", "Control", BuiltInWidgets.kTextView, 2, 0, 1, 1, 0);
        // userInterface.createComponent("Set Position (L)", "Control", BuiltInWidgets.kTextView, 2, 1, 1, 1, 0);
        // userInterface.createComponent("Set Velocity (L)", "Control", BuiltInWidgets.kTextView, 2, 2, 1, 1, 0);
        // userInterface.createComponent("Set Voltage (L)", "Control", BuiltInWidgets.kTextView, 2, 3, 1, 1, 0);

        // userInterface.createComponent("Apply Percent (L)", "Control", BuiltInWidgets.kToggleButton, 3, 0, 1, 1, false);
        // userInterface.createComponent("Apply Position (L)", "Control", BuiltInWidgets.kToggleButton, 3, 1, 1, 1, false);
        // userInterface.createComponent("Apply Velocity (L)", "Control", BuiltInWidgets.kToggleButton, 3, 2, 1, 1, false);
        // userInterface.createComponent("Apply Voltage (L)", "Control", BuiltInWidgets.kToggleButton, 3, 3, 1, 1, false);

        // userInterface.createComponent("Reset Percent (L)", "Control", BuiltInWidgets.kToggleButton, 4, 0, 1, 1, false);
        // userInterface.createComponent("Reset Velocity (L)", "Control", BuiltInWidgets.kToggleButton, 4, 2, 1, 1, false);
        // userInterface.createComponent("Reset Voltage (L)", "Control", BuiltInWidgets.kToggleButton, 4, 3, 1, 1, false);

        // userInterface.createComponent("Motor ID (F)", "Control", BuiltInWidgets.kTextView, 0, 2, 1, 1, 0);
        // userInterface.createComponent("Set (F)", "Control", BuiltInWidgets.kToggleButton, 0, 3, 1, 1, false);

        // userInterface.createComponent("Motor Percent (F)", "Control", BuiltInWidgets.kTextView, 5, 0, 1, 1, 0);
        // userInterface.createComponent("Motor Position (F)", "Control", BuiltInWidgets.kTextView, 5, 1, 1, 1, 0);
        // userInterface.createComponent("Motor Velocity (F)", "Control", BuiltInWidgets.kTextView, 5, 2, 1, 1, 0);
        // userInterface.createComponent("Motor Voltage (F)", "Control", BuiltInWidgets.kTextView, 5, 3, 1, 1, 0);

        // userInterface.createComponent("Set Percent (F)", "Control", BuiltInWidgets.kTextView, 6, 0, 1, 1, 0);
        // userInterface.createComponent("Set Position (F)", "Control", BuiltInWidgets.kTextView, 6, 1, 1, 1, 0);
        // userInterface.createComponent("Set Velocity (F)", "Control", BuiltInWidgets.kTextView, 6, 2, 1, 1, 0);
        // userInterface.createComponent("Set Voltage (F)", "Control", BuiltInWidgets.kTextView, 6, 3, 1, 1, 0);

        // userInterface.createComponent("Apply Percent (F)", "Control", BuiltInWidgets.kToggleButton, 7, 0, 1, 1, false);
        // userInterface.createComponent("Apply Position (F)", "Control", BuiltInWidgets.kToggleButton, 7, 1, 1, 1, false);
        // userInterface.createComponent("Apply Velocity (F)", "Control", BuiltInWidgets.kToggleButton, 7, 2, 1, 1, false);
        // userInterface.createComponent("Apply Voltage (F)", "Control", BuiltInWidgets.kToggleButton, 7, 3, 1, 1, false);

        // userInterface.createComponent("Reset Percent (F)", "Control", BuiltInWidgets.kToggleButton, 8, 0, 1, 1, false);
        // userInterface.createComponent("Reset Velocity (F)", "Control", BuiltInWidgets.kToggleButton, 8, 2, 1, 1, false);
        // userInterface.createComponent("Reset Voltage (F)", "Control", BuiltInWidgets.kToggleButton, 8, 3, 1, 1, false);

        // userInterface.setTab("Control");

//        LimelightHelpers.setCameraPose_RobotSpace("limelight-left",
//                -.316, -.316, .453, 0, 5, 135.218);
//        LimelightHelpers.setCameraPose_RobotSpace("limelight-right",
//                -.317, .317, .436, 180, 5, -135.218);
        Logger.addDataReceiver(new WPILOGWriter());
    }

    @Override
    public void robotPeriodic() {
        // if (userInterface.getComponentData("Set (L)").getBoolean(false)) {
        //     userInterface.setComponentData("Set (L)", false);
        //     motorLeader = new TalonFX((int) userInterface.getComponentData("Motor ID (L)").getInteger(0));
        // }

        // if (motorLeader != null) {
        //     if (motorLeader.isAlive()) {
        //         userInterface.setComponentData("Motor Percent (L)", motorLeader.get());
        //         userInterface.setComponentData("Motor Position (L)", motorLeader.getPosition().getValueAsDouble());
        //         userInterface.setComponentData("Motor Velocity (L)", motorLeader.getVelocity().getValueAsDouble());
        //         userInterface.setComponentData("Motor Voltage (L)", motorLeader.getMotorVoltage().getValueAsDouble());
        //     }
        // }

        // if (userInterface.getComponentData("Apply Percent (L)").getBoolean(false)) {
        //     userInterface.setComponentData("Apply Percent (L)", false);
        //     motorLeader.set(userInterface.getComponentData("Set Percent (L)").getDouble(0));
        // }

        // if (userInterface.getComponentData("Apply Position (L)").getBoolean(false)) {
        //     userInterface.setComponentData("Apply Position (L)", false);
        //     motorLeader.setPosition(userInterface.getComponentData("Set Position (L)").getDouble(0));
        // }

        // if (userInterface.getComponentData("Apply Velocity (L)").getBoolean(false)) {
        //     userInterface.setComponentData("Apply Velocity (L)", false);
        //     motorLeader.setControl(new MotionMagicVelocityVoltage(0).withSlot(0).withEnableFOC(true).withVelocity(userInterface.getComponentData("Set Velocity").getDouble(0)));
        // }

        // if (userInterface.getComponentData("Apply Voltage (L)").getBoolean(false)) {
        //     userInterface.setComponentData("Apply Voltage (L)", false);
        //     motorLeader.setVoltage(userInterface.getComponentData("Set Voltage (L)").getDouble(0));
        // }

        // if (userInterface.getComponentData("Reset Percent (L)").getBoolean(false)) {
        //     userInterface.setComponentData("Reset Percent (L)", false);
        //     motorLeader.set(0);
        // }

        // if (userInterface.getComponentData("Reset Velocity (L)").getBoolean(false)) {
        //     userInterface.setComponentData("Reset Velocity (L)", false);
        //     motorLeader.setControl(new MotionMagicVelocityVoltage(0).withSlot(0).withEnableFOC(true).withVelocity(0));
        // }

        // if (userInterface.getComponentData("Reset Voltage (L)").getBoolean(false)) {
        //     userInterface.setComponentData("Reset Voltage (L)", false);
        //     motorLeader.setVoltage(0);
        // }

        // if (userInterface.getComponentData("Set (F)").getBoolean(false)) {
        //     userInterface.setComponentData("Set (F)", false);
        //     motorFollower = new TalonFX((int) userInterface.getComponentData("Motor ID (F)").getInteger(0));
        // }

        // if (motorFollower != null) {
        //     if (motorFollower.isAlive()) {
        //         userInterface.setComponentData("Motor Percent (F)", motorFollower.get());
        //         userInterface.setComponentData("Motor Position (F)", motorFollower.getPosition().getValueAsDouble());
        //         userInterface.setComponentData("Motor Velocity (F)", motorFollower.getVelocity().getValueAsDouble());
        //         userInterface.setComponentData("Motor Voltage (F)", motorFollower.getMotorVoltage().getValueAsDouble());
        //     }
        // }

        // if (userInterface.getComponentData("Apply Percent (F)").getBoolean(false)) {
        //     userInterface.setComponentData("Apply Percent (F)", false);
        //     motorFollower.set(userInterface.getComponentData("Set Percent (F)").getDouble(0));
        // }

        // if (userInterface.getComponentData("Apply Position (F)").getBoolean(false)) {
        //     userInterface.setComponentData("Apply Position (F)", false);
        //     motorFollower.setPosition(userInterface.getComponentData("Set Position (F)").getDouble(0));
        // }

        // if (userInterface.getComponentData("Apply Velocity (F)").getBoolean(false)) {
        //     userInterface.setComponentData("Apply Velocity (F)", false);
        //     motorFollower.setControl(new MotionMagicVelocityVoltage(0).withSlot(0).withEnableFOC(true).withVelocity(userInterface.getComponentData("Set Velocity").getDouble(0)));
        // }

        // if (userInterface.getComponentData("Apply Voltage (F)").getBoolean(false)) {
        //     userInterface.setComponentData("Apply Voltage (F)", false);
        //     motorFollower.setVoltage(userInterface.getComponentData("Set Voltage (F)").getDouble(0));
        // }

        // if (userInterface.getComponentData("Reset Percent (F)").getBoolean(false)) {
        //     userInterface.setComponentData("Reset Percent (F)", false);
        //     motorFollower.set(0);
        // }

        // if (userInterface.getComponentData("Reset Velocity (F)").getBoolean(false)) {
        //     userInterface.setComponentData("Reset Velocity (F)", false);
        //     motorFollower.setControl(new MotionMagicVelocityVoltage(0).withSlot(0).withEnableFOC(true).withVelocity(0));
        // }

        // if (userInterface.getComponentData("Reset Voltage (F)").getBoolean(false)) {
        //     userInterface.setComponentData("Reset Voltage (F)", false);
        //     motorFollower.setVoltage(0);
        // }

        RobotContainer.periodic();
        CommandScheduler.getInstance().run();

//        Runtime runtime = Runtime.getRuntime();
//        long usedMemory = runtime.totalMemory() - runtime.freeMemory();
//        System.out.println("Used memory: " + usedMemory / 1024 / 1024 + "MB / " + runtime.totalMemory() / 1024 / 1024 + "MB");
    }

    @Override
    public void disabledInit() {
        LimelightHelpers.SetThrottle("limelight-right", 200);
        LimelightHelpers.SetThrottle("limelight-left", 200);
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void disabledExit() {
        LimelightHelpers.SetThrottle("limelight-right", 0);
        LimelightHelpers.SetThrottle("limelight-left", 0);
    }

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
    }

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
        // CommandScheduler.getInstance().schedule(new InstantCommand(() -> hoodSubsystem.getMotor().setPosition(0)));
        CommandScheduler.getInstance().schedule(new VelocityCommand(indexerSubsystem, IndexerConstants.IDLING_SPEED));
        CommandScheduler.getInstance().schedule(new VelocityCommand(hopperSubsystem, HopperConstants.IDLING_SPEED));
        // CommandScheduler.getInstance().schedule(new VelocityCommand(intakerollersubsystem, 90));

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
}
