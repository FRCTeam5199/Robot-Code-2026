package frc.robot.subsystems.templates;

import org.wpilib.command2.Command;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class HoodCommand extends Command {
    private double goal;
    private double goalVelocity;
    private HoodSubsystem hoodSubsystem;
    private boolean updateGoalPosition;

    public HoodCommand(HoodSubsystem hoodSubsystem, double goal) {
        this.hoodSubsystem = hoodSubsystem;
        this.goal = goal;
        updateGoalPosition = false;
        hoodSubsystem.setContinuousMotion(false);

        addRequirements(hoodSubsystem);
    }

    public HoodCommand(HoodSubsystem hoodSubsystem, double goal, double goalVelocity) {
        this(hoodSubsystem, goal);

        this.goalVelocity = goalVelocity;
    }

    @Override
    public void initialize() {
//        hoodSubsystem.setPositionProfiling(goal, goalVelocity);
//        hoodSubsystem.setCommandRunning(true);
//        hoodSubsystem.setStopMoving(false);
    }

    @Override
    public void execute() {
//        if (updateGoalPosition) {
//            hoodSubsystem.updateGoalPosition(goal, goalVelocity);
//            updateGoalPosition = false;
//        }
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
