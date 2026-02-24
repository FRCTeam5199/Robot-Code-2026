package frc.robot.subsystems.templates;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommand extends Command {
    private double goal;

    private ShooterSubsystem shooterSubsystem;
    private boolean updateVelocity;

    public ShooterCommand(ShooterSubsystem shooterSubsystem, double goal) {
        this.shooterSubsystem = shooterSubsystem;
        this.goal = goal;
        updateVelocity = false;

        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        if (goal == 0) shooterSubsystem.setPercent(0);
        shooterSubsystem.setVelocityBangBang(goal);
    }

    @Override
    public void execute() {
        if (updateVelocity) {
            if (goal == 0) shooterSubsystem.setPercent(0);
            shooterSubsystem.setVelocityBangBang(goal);
            updateVelocity = false;
        }
    }

    @Override
    public boolean isFinished() {
        return shooterSubsystem.isMechAtGoal();
    }

    @Override
    public void end(boolean interrupted) {
    }

    public void setGoal(double goal) {
        this.goal = goal;
        updateVelocity = true;
    }
}
