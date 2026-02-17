package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.subsystems.KickerSubsystem;

public class RobotCommands {
    private static final KickerSubsystem kickerSubsystem = KickerSubsystem.getInstance();

    public static Command spinKicker(double goalVelocity) {
        return new FunctionalCommand(
                () -> {
                    kickerSubsystem.setVelocityBangBang(goalVelocity);
                    if (kickerSubsystem.isMechAtGoal()) kickerSubsystem.setVelocityTorqueCurrent(goalVelocity);
                },
                () -> {
                    if (kickerSubsystem.isUsingBangBangControl() && kickerSubsystem.isMechAtGoal())
                        kickerSubsystem.setVelocityTorqueCurrent(goalVelocity);
                    else if (!kickerSubsystem.isUsingBangBangControl() && !kickerSubsystem.isMechAtGoal())
                        kickerSubsystem.setVelocityBangBang(goalVelocity);
                },
                (interrupted) -> {
                    kickerSubsystem.setPercent(0);
                },
                kickerSubsystem::isMechAtGoal,
                kickerSubsystem
        );
    }
}
