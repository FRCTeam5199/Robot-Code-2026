// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Arrays;
import java.util.HashSet;
import java.util.Set;
import java.util.stream.Stream;

import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.FunctionalCommand;
import org.wpilib.command2.InstantCommand;
import org.wpilib.command2.ParallelCommandGroup;
import org.wpilib.command2.button.CommandXboxController;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;

import frc.robot.constants.MiscellaneousConstants;
import frc.robot.constants.SetPoints;
import frc.robot.subsystems.Aiming;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerBottomBeltSubsystem;
import frc.robot.subsystems.IndexerTopBeltSubsystem;
import frc.robot.subsystems.IntakePivotSubsystem;
import frc.robot.subsystems.IntakeRollerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.controls.EndlessPositionCommand;
import frc.robot.subsystems.controls.EndlessVelocityCommand;
import frc.robot.subsystems.controls.MovementCommand;
import frc.robot.subsystems.controls.TemplatePositionCommand;
import frc.robot.subsystems.controls.TemplateVelocityCommand;
import frc.robot.subsystems.templates.RollerSubsystem;
import frc.robot.subsystems.templates.TemplateSubsystem;
import frc.robot.subsystems.warnings.UnimplementedWarning;
import frc.robot.subsystems.warnings.Warning;
import frc.robot.units.MechDegrees;
import frc.robot.units.MotorRotations;
import frc.robot.units.RotationalUnit;







public class RobotContainer {
	public final CommandXboxController commandXboxController;

	public final HoodSubsystem hoodSubsystem;
	public final HopperSubsystem hopperSubsystem;
	public final IndexerBottomBeltSubsystem indexerBottomBeltSubsystem;
	public final IndexerTopBeltSubsystem indexerTopBeltSubsystem;
	public final IntakePivotSubsystem intakePivotSubsystem;
	public final IntakeRollerSubsystem intakeRollerSubsystem;
	public final ShooterSubsystem shooterSubsystem;
	public final TurretSubsystem turretSubsystem;

	private final CommandSwerveDrivetrain commandSwerveDrivetrain;

	private final Set<Class<Warning>> suppressedWarnings = new HashSet<>();

	TemplatePositionCommand turretPositionCommand;
	TemplatePositionCommand hoodPositionCommand;
	TemplateVelocityCommand shooterVelocityCommand;	

	private final Aiming aiming;
	private TemplateSubsystem[] subsystems;

	private final Vision vision;

	public RobotContainer() {
		commandXboxController = new CommandXboxController(MiscellaneousConstants.XBOX_PORT);
		configureBindings();

		// shorthands to make subsystem usage more concise
		hoodSubsystem = HoodSubsystem.getInstance();
		hopperSubsystem = HopperSubsystem.getInstance();
		indexerBottomBeltSubsystem = IndexerBottomBeltSubsystem.getInstance();
		indexerTopBeltSubsystem = IndexerTopBeltSubsystem.getInstance();
		intakePivotSubsystem = IntakePivotSubsystem.getInstance();
		intakeRollerSubsystem = IntakeRollerSubsystem.getInstance();
		shooterSubsystem = ShooterSubsystem.getInstance();
		turretSubsystem = TurretSubsystem.getInstance();

		vision = Vision.getInstance();
		vision.setController(this);

		turretPositionCommand = new EndlessPositionCommand(turretSubsystem, new MechDegrees(0));
		hoodPositionCommand = new EndlessPositionCommand(hoodSubsystem, new MechDegrees(0));
		shooterVelocityCommand = new EndlessVelocityCommand(shooterSubsystem, new MotorRotations(0));


		aiming = new Aiming(this);


		subsystems = new TemplateSubsystem[]{intakePivotSubsystem, turretSubsystem, hoodSubsystem, indexerBottomBeltSubsystem, indexerTopBeltSubsystem, hopperSubsystem, intakeRollerSubsystem, shooterSubsystem};
	}

	private int subsystemID;

	private void configureBindings() {
		commandXboxController.dpadUp().onTrue(new InstantCommand(() -> {subsystemID++; if(subsystemID >= subsystems.length) subsystemID = subsystems.length-1;}));
		commandXboxController.dpadDown().onTrue(new InstantCommand(() -> {subsystemID--; if(subsystemID < 0) subsystemID = 0;}));
		commandXboxController.a().onTrue(new InstantCommand(() -> {
			if(subsystems[subsystemID] instanceof RollerSubsystem){
				subsystems[subsystemID].setVelocity(new MotorRotations(10));
			}else{
				subsystems[subsystemID].setPercentage(0.15);
			}
		}));
		commandXboxController.b().onTrue(new InstantCommand(() -> {
			if(subsystems[subsystemID] instanceof RollerSubsystem){
				subsystems[subsystemID].setVelocity(new MotorRotations(0));
			}else{
				subsystems[subsystemID].setPercentage(0);
			}
		}));
		// commandXboxController.dpadDown().onTrue(new TemplatePositionCommand(intakePivotSubsystem, new MechDegrees(30)));
		// commandXboxController.dpadUp().onTrue(new TemplatePositionCommand(intakePivotSubsystem, new MechDegrees(0)));

		
		// commandXboxController.x().onTrue(new InstantCommand(() -> aiming.setPoint(SetPoints.POINT1)));
		// commandXboxController.y().onTrue(new InstantCommand(() -> aiming.setPoint(SetPoints.POINT2)));
		// commandXboxController.a().onTrue(new InstantCommand(() -> aiming.setPoint(SetPoints.POINT3)));
		// commandXboxController.b().onTrue(new InstantCommand(() -> aiming.setPoint(SetPoints.POINT4)));
		
		// // enable autoaim
		// commandXboxController.leftTrigger().onTrue(new InstantCommand(() -> aiming.setAutoAimEnabled(true)))
		// 						 .onFalse(new InstantCommand(() -> aiming.setAutoAimEnabled(false)));
		
		// commandXboxController.leftBumper().onTrue(new TemplateVelocityCommand(intakeRollerSubsystem, new MotorRotations(50)))
		// 								  .onFalse(new TemplateVelocityCommand(intakeRollerSubsystem, new MotorRotations(0)));


		// MovementCommand[] setupCommands = new MovementCommand[]{
		// 	turretPositionCommand,
		// 	hoodPositionCommand,
		// 	shooterVelocityCommand
		// };

		// commandXboxController.rightBumper().onTrue(
		// 	new ParallelCommandGroup(
		// 		turretPositionCommand,
		// 		hoodPositionCommand,
		// 		shooterVelocityCommand,
		// 		new FunctionalCommand(
		// 			() -> {},
		// 			() -> {
		// 				if(Arrays.stream(setupCommands).anyMatch((cmd) -> !cmd.isMechAtGoal())){
		// 					indexerBottomBeltSubsystem.setVelocity(new MotorRotations(0));
		// 					indexerTopBeltSubsystem.setVelocity(new MotorRotations(0));
		// 					hopperSubsystem.setVelocity(new MotorRotations(0));
		// 				}
		// 				else{
		// 					indexerBottomBeltSubsystem.setVelocity(new MotorRotations(100));
		// 					indexerTopBeltSubsystem.setVelocity(new MotorRotations(100));
		// 					hopperSubsystem.setVelocity(new MotorRotations(100));
		// 				}
		// 			}, 
		// 			(interrupted) -> {},
		// 			() -> {return false;}, 
		// 		indexerBottomBeltSubsystem, indexerTopBeltSubsystem, hopperSubsystem)
		// 	)
		// ).onFalse(
		// 	new ParallelCommandGroup(
		// 		new TemplatePositionCommand(turretSubsystem, new MechDegrees(0)),
		// 		new TemplatePositionCommand(hoodSubsystem, new MechDegrees(0)),
		// 		new TemplateVelocityCommand(shooterSubsystem, new MotorRotations(0)),

		// 		new TemplateVelocityCommand(indexerBottomBeltSubsystem, new MotorRotations(0)),
		// 		new TemplateVelocityCommand(indexerTopBeltSubsystem, new MotorRotations(0)),
		// 		new TemplateVelocityCommand(hopperSubsystem, new MotorRotations(0))
		// 	)
		// );
	}

	public void suppressWarning(Class<Warning> warning){
		this.suppressedWarnings.add(warning);
	}

	public void emitWarning(Warning warning){
		if(this.suppressedWarnings.contains(warning.getClass())) return;
		System.err.println("[WARNING "+getClass().getName()+"] "+warning.message);
	}

	public void updateTurretCommandPositions(RotationalUnit turretRotation, RotationalUnit hoodRotation, RotationalUnit shooterVelocity){
		turretPositionCommand.updatePosition(turretRotation);
		hoodPositionCommand.updatePosition(hoodRotation);
		shooterVelocityCommand.updateVelocity(shooterVelocity);
	}

	public void periodic(){
		// aiming.update();
	}

	public Command getAutonomousCommand() {
		return Commands.print("No autonomous command configured");
	}

	public Pose2d getPose() {
		emitWarning(new UnimplementedWarning("RobotContainer.getPose is unimplemented; please use the motor odometers from the drive tracks"));
		return new Pose2d(0, 0, new Rotation2d(0));
	}

	public CommandSwerveDrivetrain getCommandSwerveDrivetrain(){
		return this.commandSwerveDrivetrain;
	}

	public boolean getState(){
		return true;
	}
}
