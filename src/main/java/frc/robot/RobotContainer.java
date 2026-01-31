// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.Constants;
import frc.robot.constants.IntakePivotConstants;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.templates.PositionCommand;
import frc.robot.subsystems.templates.VelocityCommand;
import frc.robot.utility.Setpoint;

import java.util.Map;


public class RobotContainer {
    public static double MaxSpeed = TunerConstants.kSpeedAt12Volts.baseUnitMagnitude(); // kSpeedAt12VoltsMps desired top speed
    public static double MaxAngularRate = 2.5 * Math.PI; //Originally 2 * Math.PI
    public final static SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric().withDesaturateWheelSpeeds(true)
            .withDeadband(MaxSpeed * .05).withRotationalDeadband(MaxAngularRate * .05) // Add a 10% deadband
            .withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.OpenLoopVoltage);
    public static Telemetry logger = new Telemetry(MaxSpeed);

    private static Setpoint currentSetpoint = Setpoint.HUB;
    CommandXboxController commandXboxController = new CommandXboxController(Constants.XBOX_PORT);
    IntakeRollerSubsystem intakeRollerSubsystem = IntakeRollerSubsystem.getInstance();
    IntakePivotSubsystem intakePivotSubsystem = IntakePivotSubsystem.getInstance();
    HopperSubsystem hopperSubsystem = HopperSubsystem.getInstance();
    ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();
    KickerSubsystem kickerSubsystem = KickerSubsystem.getInstance();
    HoodSubsystem hoodSubsystem = HoodSubsystem.getInstance();
    TurretSubsystem turretSubsystem = TurretSubsystem.getInstance();
    CommandSwerveDrivetrain commandSwerveDrivetrain = TunerConstants.createDrivetrain();

    public RobotContainer() {
        configureBindings();
    }

    public static Setpoint getCurrentSetpoint() {
        return currentSetpoint;
    }

    public static void setCurrentSetpoint(Setpoint currentSetpoint) {
        RobotContainer.currentSetpoint = currentSetpoint;
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
                .onTrue(new PositionCommand(intakePivotSubsystem, IntakePivotConstants.INTAKE_OUT, IntakePivotConstants.INTAKE_PIVOT_VELOCITY_OUT, IntakePivotConstants.INTAKE_PIVOT_ACCELERATION_OUT, IntakePivotConstants.INTAKE_PIVOT_JERK_OUT)
                        .alongWith(new VelocityCommand(intakeRollerSubsystem, 60))
                        .alongWith(new VelocityCommand(hopperSubsystem, -10)))
                .onFalse(new PositionCommand(intakePivotSubsystem, IntakePivotConstants.INTAKE_IN, IntakePivotConstants.INTAKE_PIVOT_VELOCITY_IN, IntakePivotConstants.INTAKE_PIVOT_ACCELERATION_IN, IntakePivotConstants.INTAKE_PIVOT_JERK_IN)
                        .alongWith(new VelocityCommand(intakeRollerSubsystem, 0))
                        .alongWith(new VelocityCommand(hopperSubsystem, 0)));

        commandXboxController.leftTrigger().onTrue(
                new SelectCommand<>(Map.ofEntries(
                        Map.entry(Setpoint.HUB, new ParallelCommandGroup(
                                new PositionCommand(turretSubsystem, Setpoint.HUB.getTurretAngle()),
                                new PositionCommand(hoodSubsystem, Setpoint.HUB.getHoodAngle()),
                                new VelocityCommand(shooterSubsystem, Setpoint.HUB.getShooterSpeed())
                        )),
                        Map.entry(Setpoint.TOWER, new ParallelCommandGroup(
                                new PositionCommand(turretSubsystem, Setpoint.TOWER.getTurretAngle()),
                                new PositionCommand(hoodSubsystem, Setpoint.TOWER.getHoodAngle()),
                                new VelocityCommand(shooterSubsystem, Setpoint.TOWER.getShooterSpeed())
                        )),
                        Map.entry(Setpoint.OUTPOST, new ParallelCommandGroup(
                                new PositionCommand(turretSubsystem, Setpoint.OUTPOST.getTurretAngle()),
                                new PositionCommand(hoodSubsystem, Setpoint.OUTPOST.getHoodAngle()),
                                new VelocityCommand(shooterSubsystem, Setpoint.OUTPOST.getShooterSpeed())
                        )),
                        Map.entry(Setpoint.LEFT_CORNER, new ParallelCommandGroup(
                                new PositionCommand(turretSubsystem, Setpoint.LEFT_CORNER.getTurretAngle()),
                                new PositionCommand(hoodSubsystem, Setpoint.LEFT_CORNER.getHoodAngle()),
                                new VelocityCommand(shooterSubsystem, Setpoint.LEFT_CORNER.getShooterSpeed())
                        ))
                ), RobotContainer::getCurrentSetpoint)
        ).onFalse(
                new ParallelCommandGroup(
                        new PositionCommand(turretSubsystem, 0),
                        new PositionCommand(hoodSubsystem, 0),
                        new VelocityCommand(shooterSubsystem, 0)
                )
        );

        commandXboxController.rightBumper().onTrue(new VelocityCommand(kickerSubsystem, 50)
                        .alongWith(new WaitUntilCommand(() -> kickerSubsystem.isMechAtGoal(true))
                                .andThen(new VelocityCommand(hopperSubsystem, 25)))) //25
                .onFalse(new VelocityCommand(kickerSubsystem, 0)
                        .alongWith(new VelocityCommand(hopperSubsystem, 0)));

        commandXboxController.leftBumper().onTrue(
                new PositionCommand(intakePivotSubsystem, 107));

        commandXboxController.y().onTrue(new InstantCommand(() -> setCurrentSetpoint(Setpoint.HUB)));
        commandXboxController.x().onTrue(new InstantCommand(() -> setCurrentSetpoint(Setpoint.LEFT_CORNER)));
        commandXboxController.b().onTrue(new InstantCommand(() -> setCurrentSetpoint(Setpoint.OUTPOST)));
        commandXboxController.a().onTrue(new InstantCommand(() -> setCurrentSetpoint(Setpoint.TOWER)));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
