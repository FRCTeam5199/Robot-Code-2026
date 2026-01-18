package frc.robot.subsystems.templates;

import edu.wpi.first.wpilibj2.command.Command;

public class PositionCommand extends Command {
    private double velocity;
    private double acceleration;
    private double jerk;
    private double goal;
    private TemplateSubsystem templateSubsystem;
    private boolean updateGoalPosition;
    private boolean changeConstraint;

    public PositionCommand(TemplateSubsystem templateSubsystem, double goal) {
        this.templateSubsystem = templateSubsystem;
        this.goal = goal;
        updateGoalPosition = false;
        changeConstraint = false;

        addRequirements(templateSubsystem);
    }

    public PositionCommand(TemplateSubsystem templateSubsystem, double goal,
                           double velocity, double acceleration, double jerk) {
        this.templateSubsystem = templateSubsystem;
        this.goal = goal;
        updateGoalPosition = false;

        this.velocity = velocity;
        this.acceleration = acceleration;
        this.jerk = jerk;

        changeConstraint = true;

        addRequirements(templateSubsystem);
    }

    @Override
    public void initialize() {
        if (changeConstraint) {
            templateSubsystem.setPosition(goal, velocity, acceleration, jerk);
            changeConstraint = false;
        } else {
            templateSubsystem.setPosition(goal);
        }
        templateSubsystem.setCommandRunning(true);
    }

    @Override
    public void execute() {
        if (updateGoalPosition) {
            templateSubsystem.setPosition(goal);
            updateGoalPosition = false;
        }
        if (changeConstraint){
            templateSubsystem.setConstraints(velocity, acceleration, jerk);
            changeConstraint = false;
        }
    }


    @Override
    public boolean isFinished() {
        return templateSubsystem.isMechAtGoal(false);
    }

    @Override
    public void end(boolean interrupted) {
        templateSubsystem.setFollowLastMechProfile(true);
        templateSubsystem.setCommandRunning(false);
    }

    public void setGoal(double goal) {
        this.goal = goal;
        this.updateGoalPosition = true;
    }

    public void setConstraints(double velocity, double acceleration, double jerk) {
        this.velocity = velocity;
        this.acceleration = acceleration;
        this.jerk = jerk;

        this.changeConstraint = true;
    }
}
