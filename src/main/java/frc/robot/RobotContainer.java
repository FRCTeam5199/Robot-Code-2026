// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import java.util.Map;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.Constants;
import frc.robot.constants.IntakePivotConstants;
import frc.robot.constants.KickerConstants;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakePivotSubsystem;
import frc.robot.subsystems.IntakeRollerSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.templates.PositionCommand;
// import frc.robot.subsystems.templates.ShooterCommand;
//import frc.robot.subsystems.templates.ShooterCommand;
import frc.robot.subsystems.templates.TurretCommand;
import frc.robot.subsystems.templates.VelocityCommand;
import frc.robot.utility.Setpoint;
import frc.robot.utility.ShotCalculator;

import javax.print.attribute.SetOfIntegerSyntax;

public class RobotContainer {
    public static final CommandXboxController commandXboxController = new CommandXboxController(Constants.XBOX_PORT);
    public static final CommandSwerveDrivetrain commandSwerveDrivetrain = TunerConstants.createDrivetrain();
    public static final IntakeRollerSubsystem intakeRollerSubsystem = IntakeRollerSubsystem.getInstance();
    public static final IntakePivotSubsystem intakePivotSubsystem = IntakePivotSubsystem.getInstance();
    public static final HopperSubsystem hopperSubsystem = HopperSubsystem.getInstance();
    public static final ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();
    public static final KickerSubsystem kickerSubsystem = KickerSubsystem.getInstance();
    public static final HoodSubsystem hoodSubsystem = HoodSubsystem.getInstance();
    public static final ShotCalculator shotCalculator = ShotCalculator.getInstance();
    public static final TurretSubsystem turretSubsystem = TurretSubsystem.getInstance();
    public static final IndexerSubsystem indexerSubsystem = IndexerSubsystem.getInstance();
    public static double MaxSpeed = TunerConstants.kSpeedAt12Volts.baseUnitMagnitude(); // kSpeedAt12VoltsMps desired top speed
    public static double MaxAngularRate = 2.5 * Math.PI; //Originally 2 * Math.PI
    public final static SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric().withDesaturateWheelSpeeds(true)
            .withDeadband(MaxSpeed * .05).withRotationalDeadband(MaxAngularRate * .05) // Add a 10% deadband
            .withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.OpenLoopVoltage);
    public static Telemetry logger = new Telemetry(MaxSpeed);
    private static Setpoint currentSetpoint = Setpoint.HUB;
    private static TurretCommand turretControl = new TurretCommand(turretSubsystem, 0, 0);
    //    private static PositionCommand hoodControl = new PositionCommand(hoodSubsystem, 0, true, false, 0);
    private static VelocityCommand shooterSpeed = new VelocityCommand(shooterSubsystem, 0);
    private static VelocityCommand kickerSpeed = new VelocityCommand(kickerSubsystem, 0);

    public RobotContainer() {
        configureBindings();
    }

    public static Setpoint getCurrentSetpoint() {
        return currentSetpoint;
    }

    public static void setCurrentSetpoint(Setpoint currentSetpoint) {
        RobotContainer.currentSetpoint = currentSetpoint;
    }

    public static void periodic() {
        turretControl.setGoal(shotCalculator.getTurretAngle(),
                turretSubsystem.getMotorRotFromDegrees(shotCalculator.getTurretVelocity()));

        shooterSpeed.setGoal(shotCalculator.getShooterSpeed());
        kickerSpeed.setGoal(shotCalculator.getKickerSpeed());
        logger.telemeterize(commandSwerveDrivetrain.getState());
    }

    public static boolean areMechanismsAtGoals() {
        return turretSubsystem.isMechAtGoal() && hoodSubsystem.isMechAtGoal(false)
                && shooterSubsystem.isMechAtGoal(true)
                && kickerSubsystem.isMechAtGoal(true);
    }

    private void configureBindings() {
        commandSwerveDrivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
                commandSwerveDrivetrain.applyRequest(() -> drive.withVelocityX(-commandXboxController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                        .withVelocityY(-commandXboxController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(-commandXboxController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
                ));

        commandXboxController.button(8).onTrue(commandSwerveDrivetrain
                .runOnce(commandSwerveDrivetrain::seedFieldCentric));
        commandXboxController.button(7)
                .onTrue(new InstantCommand(() -> commandSwerveDrivetrain.getPigeon2().setYaw(0)));

//         commandXboxController.povRight().onTrue(new PositionCommand(turretSubsystem, 0)
//                 .andThen(new InstantCommand(() -> CommandScheduler.getInstance().cancelAll()))
//                 .andThen(new WaitCommand(.5))
//                 .andThen(new InstantCommand(() -> turretSubsystem.resetMotorOnSometimesEncoder()))
//                 .andThen(new InstantCommand(() -> CommandScheduler.getInstance().cancelAll())));

        commandXboxController.leftBumper().onTrue(new ParallelCommandGroup(shooterSpeed, kickerSpeed));

        commandXboxController.povDown().onTrue(
                new PositionCommand(intakePivotSubsystem, IntakePivotConstants.INTAKE_OUT));

        commandXboxController.povUp().onTrue(
                new PositionCommand(intakePivotSubsystem, IntakePivotConstants.INTAKE_IN));

        commandXboxController.rightTrigger().onTrue(new VelocityCommand(intakeRollerSubsystem, 90))
                .onFalse(new VelocityCommand(intakeRollerSubsystem, 0));
//        commandXboxController.rightTrigger()
//                .onTrue(new PositionCommand(intakePivotSubsystem, IntakePivotConstants.INTAKE_OUT)
//                        .alongWith(new VelocityCommand(intakeRollerSubsystem, 90)))
//                .onFalse(new PositionCommand(intakePivotSubsystem, IntakePivotConstants.INTAKE_IN)
//                        .alongWith(new VelocityCommand(intakeRollerSubsystem, 0)));

        commandXboxController.leftTrigger().onTrue(new ParallelCommandGroup(
                new VelocityCommand(indexerSubsystem, 30),
                new VelocityCommand(hopperSubsystem, 60)
        )).onFalse(
                new ParallelCommandGroup(
                        new VelocityCommand(indexerSubsystem, -15),
                        new VelocityCommand(hopperSubsystem, -5)));

//        commandXboxController.leftTrigger().onTrue(
//                        new SelectCommand<>(Map.ofEntries(
//                                Map.entry(Setpoint.HUB, new ParallelCommandGroup(
//                                        new TurretCommand(turretSubsystem, Setpoint.HUB.getTurretAngle(), 0),
//                                        new PositionCommand(hoodSubsystem, Setpoint.HUB.getHoodAngle()),
//                                        new VelocityCommand(shooterSubsystem, Setpoint.HUB.getShooterSpeed()),
//                                        new VelocityCommand(kickerSubsystem, Setpoint.HUB.getShooterSpeed())
//                                )),
//                                Map.entry(Setpoint.TOWER, new ParallelCommandGroup(
//                                        new TurretCommand(turretSubsystem, Setpoint.TOWER.getTurretAngle(), 0),
//                                        new PositionCommand(hoodSubsystem, Setpoint.TOWER.getHoodAngle()),
//                                        new VelocityCommand(shooterSubsystem, Setpoint.TOWER.getShooterSpeed()),
//                                        new VelocityCommand(kickerSubsystem, Setpoint.TOWER.getShooterSpeed())
//                                )),
//                                Map.entry(Setpoint.OUTPOST, new ParallelCommandGroup(
//                                        new TurretCommand(turretSubsystem, Setpoint.OUTPOST.getTurretAngle(), 0),
//                                        new PositionCommand(hoodSubsystem, Setpoint.OUTPOST.getHoodAngle()),
//                                        new VelocityCommand(shooterSubsystem, Setpoint.OUTPOST.getShooterSpeed()),
//                                        new VelocityCommand(kickerSubsystem, Setpoint.OUTPOST.getShooterSpeed())
//                                )),
//                                Map.entry(Setpoint.LEFT_CORNER, new ParallelCommandGroup(
//                                        new TurretCommand(turretSubsystem, Setpoint.LEFT_CORNER.getTurretAngle(), 0),
//                                        new PositionCommand(hoodSubsystem, Setpoint.LEFT_CORNER.getHoodAngle()),
//                                        new VelocityCommand(shooterSubsystem, Setpoint.LEFT_CORNER.getShooterSpeed()),
//                                        new VelocityCommand(kickerSubsystem, Setpoint.LEFT_CORNER.getShooterSpeed())
//                                ))
//                        ), RobotContainer::getCurrentSetpoint).alongWith(RobotCommands.indexBalls()))
//                .onFalse(
//                        new ParallelCommandGroup(
//                                RobotCommands.zeroTurret(),
//                                new PositionCommand(hoodSubsystem, 0),
//                                new VelocityCommand(shooterSubsystem, 0),
//                                new VelocityCommand(indexerSubsystem, -15),
//                                new VelocityCommand(kickerSubsystem, 0, 0),
//                                new VelocityCommand(hopperSubsystem, -5)
//                        )
//                );


//        commandXboxController.y().onTrue(new InstantCommand(() -> setCurrentSetpoint(Setpoint.HUB)));
//        commandXboxController.x().onTrue(new InstantCommand(() -> setCurrentSetpoint(Setpoint.LEFT_CORNER)));
//        commandXboxController.b().onTrue(new InstantCommand(() -> setCurrentSetpoint(Setpoint.OUTPOST)));
//        commandXboxController.a().onTrue(new InstantCommand(() -> setCurrentSetpoint(Setpoint.TOWER)));

        // commandXboxController.a().onTrue(
        //         new SequentialCommandGroup(
        //                 new PositionCommand(intakePivotSubsystem, IntakePivotConstants.INTAKE_OUT),
        //                 new PositionCommand(intakePivotSubsystem, 80)
        //         ).repeatedly()
        // );
        commandXboxController.povLeft().onTrue(RobotCommands.zeroTurret());

        commandXboxController.povRight().onTrue(turretControl);
//        commandXboxController.povLeft().onTrue(new PositionCommand(hoodSubsystem, 0));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
