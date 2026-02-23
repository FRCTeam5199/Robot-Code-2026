package frc.robot.subsystems.templates;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem;

public class TurretCommand extends Command {
    private double goal;
    private double goalVelocity;
    private TurretSubsystem turretSubsystem;
    private boolean updateGoalPosition;

    public TurretCommand(TurretSubsystem turretSubsystem, double goal) {
        this.turretSubsystem = turretSubsystem;
        this.goal = goal;
        updateGoalPosition = false;
        turretSubsystem.setContinuousMotion(false);

        addRequirements(turretSubsystem);
    }

    public TurretCommand(TurretSubsystem turretSubsystem, double goal, double goalVelocity) {
        this(turretSubsystem, goal);

        this.goalVelocity = goalVelocity;
    }

    @Override
    public void initialize() {
        turretSubsystem.setPositionProfiling(goal, goalVelocity);
        turretSubsystem.setCommandRunning(true);
        turretSubsystem.setStopMoving(false);
    }

    @Override
    public void execute() {
        if (updateGoalPosition) {
            turretSubsystem.updateGoalPosition(goal, goalVelocity);
            updateGoalPosition = false;
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
    }

    public void setGoal(double goal) {
        this.goal = goal;
        this.updateGoalPosition = true;
    }

    public void setGoal(double goal, double velocity) {
        this.goalVelocity = velocity;
        setGoal(goal);
    }
}