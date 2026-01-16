// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.Constants;
import frc.robot.subsystems.Subsystem;



public class RobotContainer {
    CommandXboxController commandXboxController = new  CommandXboxController(Constants.xbox);

  public RobotContainer() {
    configureBindings();
  }

  Subsystem subsystem = new Subsystem();

  private void configureBindings() {
    commandXboxController.a().onTrue(new InstantCommand(() -> subsystem.setMotionMagicVelocity(100)))
    .onFalse(new InstantCommand(() -> subsystem.setMotionMagicVelocity(0)));
    commandXboxController.b().onTrue(new InstantCommand(() -> subsystem.setVelocityVoltage(100)))
    .onFalse(new InstantCommand(() -> subsystem.setVelocityVoltage(0)));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
