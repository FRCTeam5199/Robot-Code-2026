package frc.robot.subsystems.templates;

import org.wpilib.command2.Command;

public class PositionCommand extends Command {
    private double velocity;
    private double acceleration;
    private double jerk;
    private double goal;
    private double goalVelocity;
    private TemplateSubsystem templateSubsystem;
    private boolean updateGoalPosition;
    private boolean changeConstraints;
    private boolean uniqueConstraints;
    private boolean shouldNotEnd;
    private boolean useCustomFF;

    public PositionCommand(TemplateSubsystem templateSubsystem, double goal) {
        this.templateSubsystem = templateSubsystem;
        this.goal = goal;
        updateGoalPosition = false;
        changeConstraints = false;
        uniqueConstraints = false;

        addRequirements(templateSubsystem);
    }

    public PositionCommand(TemplateSubsystem templateSubsystem, double goal, boolean shouldNotEnd,
                           boolean useCustomFF, double goalVelocity) {
        this(templateSubsystem, goal);
        this.shouldNotEnd = shouldNotEnd;
        this.useCustomFF = useCustomFF;
        this.goalVelocity = goalVelocity;
    }

    public PositionCommand(TemplateSubsystem templateSubsystem, double goal,
                           double velocity, double acceleration, double jerk) {
        this.templateSubsystem = templateSubsystem;
        this.goal = goal;
        updateGoalPosition = false;

        this.velocity = velocity;
        this.acceleration = acceleration;
        this.jerk = jerk;

        changeConstraints = true;
        uniqueConstraints = true;

        addRequirements(templateSubsystem);
    }

    @Override
    public void initialize() {
        if (changeConstraints) {
            templateSubsystem.setPosition(goal);
            templateSubsystem.setConstraints(velocity, acceleration, jerk);
            changeConstraints = false;
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
        if (changeConstraints) {
            templateSubsystem.setConstraints(velocity, acceleration, jerk);
            changeConstraints = false;
        }
    }

    @Override
    public boolean isFinished() {
        return templateSubsystem.isMechAtGoal(false) && !shouldNotEnd;
    }

    @Override
    public void end(boolean interrupted) {
        templateSubsystem.setFollowLastMechProfile(true);
        templateSubsystem.setCommandRunning(false);
        if (uniqueConstraints) changeConstraints = true;
    }

    public void setGoal(double goal) {
        this.goal = goal;
        this.updateGoalPosition = true;
    }

    public void setGoal(double goal, double velocity) {
        this.goalVelocity = velocity;
        setGoal(goal);
    }

    public void setConstraints(double velocity, double acceleration, double jerk) {
        this.velocity = velocity;
        this.acceleration = acceleration;
        this.jerk = jerk;

        this.changeConstraints = true;
    }
}
