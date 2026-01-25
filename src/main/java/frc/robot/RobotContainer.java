// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.Constants;
import frc.robot.generated.IntakePivotConstants;
import frc.robot.generated.IntakeRollerConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakePivotSubsystem;
import frc.robot.subsystems.IntakeRollerSubsystem;
import frc.robot.subsystems.templates.PositionCommand;
import frc.robot.subsystems.templates.VelocityCommand;



public class RobotContainer {
  CommandXboxController commandXboxController = new CommandXboxController(Constants.XBOX_PORT);
  IntakeRollerSubsystem intakeRollerSubsystem = IntakeRollerSubsystem.getInstance();
  IntakePivotSubsystem intakePivotSubsystem = IntakePivotSubsystem.getInstance();
  CommandSwerveDrivetrain commandSwerveDrivetrain = TunerConstants.createDrivetrain();
  HopperSubsystem hopperSubsystem = HopperSubsystem.getInstance();

  public static double MaxSpeed = TunerConstants.kSpeedAt12Volts.baseUnitMagnitude(); // kSpeedAt12VoltsMps desired top speed
  public static double MaxAngularRate = 2.5 * Math.PI; //Originally 2 * Math.PI

  public final static SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric().withDesaturateWheelSpeeds(true)
          .withDeadband(MaxSpeed * .05).withRotationalDeadband(MaxAngularRate * .05) // Add a 10% deadband
          .withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.OpenLoopVoltage);

  public static Telemetry logger = new Telemetry(MaxSpeed);

  public RobotContainer() {
    configureBindings();
  }


  private void configureBindings() {
    commandSwerveDrivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
            commandSwerveDrivetrain.applyRequest(() -> drive.withVelocityX(-commandXboxController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-commandXboxController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-commandXboxController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            ));

        commandXboxController.button(8).onTrue(commandSwerveDrivetrain
            .runOnce(commandSwerveDrivetrain::seedFieldCentric)
            .alongWith(new InstantCommand(() -> commandSwerveDrivetrain.getPigeon2().setYaw(0))));

    commandXboxController.rightTrigger()
            .onTrue(new PositionCommand(intakePivotSubsystem, IntakePivotConstants.INTAKE_OUT)
                    .alongWith(new VelocityCommand(intakeRollerSubsystem, 30)))
            .onFalse(new PositionCommand(intakePivotSubsystem, IntakePivotConstants.INTAKE_IN)
                    .alongWith(new VelocityCommand(intakeRollerSubsystem, 0)));

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
