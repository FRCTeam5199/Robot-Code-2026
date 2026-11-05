package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.RobotContainer;
import frc.robot.constants.IntakeLinearConstants;
import frc.robot.constants.KickerConstants;
import frc.robot.subsystems.IntakeLinearSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.templates.MotorGroup;
import frc.robot.subsystems.templates.TemplateSubsystem;

public class RobotCommands {
    public static final KickerSubsystem kickerSubsystem = KickerSubsystem.getInstance();


    public static Command indexBallsAuto() {
        return new FunctionalCommand(
                () -> {
                    if (RobotContainer.areMechanismsAtGoalsAuto()) {
                        kickerSubsystem.getMotorGroup().setVelocity(KickerConstants.KICKER_SPEED);
                    }
                },
                () -> {
                    if (RobotContainer.areMechanismsAtGoalsExeptShooterAuto()) {
                        kickerSubsystem.getMotorGroup().setVelocity(KickerConstants.KICKER_SPEED);
                    } else {
                        kickerSubsystem.getMotorGroup().setVelocity(KickerConstants.KICKER_REVERSE);
                    }

                },
                (interrupted) -> {
                    kickerSubsystem.getMotorGroup().setVelocity(KickerConstants.KICKER_REVERSE);
                },
                () -> false,
                kickerSubsystem
        );
    }

    public static final IntakeLinearSubsystem intakeLinearSubsystem = IntakeLinearSubsystem.getInstance();

    public static final int StalinTol = 10;

    public static Command intakeOut() {
        MotorGroup left = intakeLinearSubsystem.getLeft();
        MotorGroup right = intakeLinearSubsystem.getRight();

        return new FunctionalCommand(
                () -> {
                        left.setPosition(IntakeLinearConstants.DEPLOY);
                        right.setPosition(IntakeLinearConstants.DEPLOY);
                },
                () -> {
                    if (RobotContainer.getStallinL()>=StalinTol) {
                        left.setPosition(left.getMechM()-.1);
                    }
                    if (RobotContainer.getStallinR()>=StalinTol) {
                        right.setPosition(right.getMechM()-.1);
                    }

                },
                (interrupted) -> {
                },
                () -> false,
                intakeLinearSubsystem
        );
    }
    
}
