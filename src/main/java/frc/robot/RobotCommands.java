package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.RobotContainer;
import frc.robot.constants.IntakeLinearConstants;
import frc.robot.constants.KickerConstants;
import frc.robot.subsystems.IntakeLinearLeft;
import frc.robot.subsystems.IntakeLinearRight;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.templates.TemplateSubsystem;

public class RobotCommands {
    public static final KickerSubsystem kickerSubsystem = KickerSubsystem.getInstance();


    public static Command indexBallsAuto() {
        return new FunctionalCommand(
                () -> {
                    if (RobotContainer.areMechanismsAtGoalsAuto()) {
                        kickerSubsystem.setVelocity(KickerConstants.KICKER_SPEED);
                    }
                },
                () -> {
                    if (RobotContainer.areMechanismsAtGoalsExeptShooterAuto()) {
                        kickerSubsystem.setVelocity(KickerConstants.KICKER_SPEED);
                    } else {
                        kickerSubsystem.setVelocity(KickerConstants.KICKER_REVERSE);
                    }

                },
                (interrupted) -> {
                    kickerSubsystem.setVelocity(KickerConstants.KICKER_REVERSE);
                },
                () -> false,
                kickerSubsystem
        );
    }

    public static final IntakeLinearRight intakeLinearRSubsystem = IntakeLinearRight.getInstance();
    public static final IntakeLinearLeft intakeLinearLSubsystem = IntakeLinearLeft.getInstance();

    public static final int StalinTol = 10;

    public static Command intakeOut() {
        return new FunctionalCommand(
                () -> {
                        intakeLinearLSubsystem.setPosition(IntakeLinearConstants.DEPLOY);
                        intakeLinearRSubsystem.setPosition(IntakeLinearConstants.DEPLOY);
                },
                () -> {
                    if (RobotContainer.getStallinL()>=StalinTol) {
                        intakeLinearLSubsystem.setPosition(intakeLinearLSubsystem.getMechM()-.1);
                    }
                    if (RobotContainer.getStallinR()>=StalinTol) {
                        intakeLinearRSubsystem.setPosition(intakeLinearRSubsystem.getMechM()-.1);
                    }

                },
                (interrupted) -> {
                },
                () -> false,
                intakeLinearLSubsystem, intakeLinearRSubsystem
        );
    }
    
}
