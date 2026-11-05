package frc.robot.subsystems.templates;

import java.util.LinkedHashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * Moves motor groups to set positions, and finishes when all of them are at their goals. A command
 * requires its whole subsystem, so to move several groups of one subsystem together, use
 * {@link #withGoal} instead of two separate commands.
 */
public class PositionCommand extends Command {
    private final TemplateSubsystem templateSubsystem;
    private final MotorGroup motorGroup;
    private final Map<MotorGroup, Double> goals = new LinkedHashMap<>();
    private double velocity;
    private double acceleration;
    private double jerk;
    private double goalVelocity;
    private boolean updateGoalPosition;
    private boolean changeConstraints;
    private boolean uniqueConstraints;
    private boolean shouldNotEnd;

    /** Moves the subsystem's first motor group. */
    public PositionCommand(TemplateSubsystem templateSubsystem, double goal) {
        this(templateSubsystem, templateSubsystem.getMotorGroup(), goal);
    }

    /** Moves a specific motor group of the subsystem. */
    public PositionCommand(TemplateSubsystem templateSubsystem, MotorGroup motorGroup, double goal) {
        this.templateSubsystem = templateSubsystem;
        this.motorGroup = motorGroup;
        goals.put(motorGroup, goal);
        updateGoalPosition = false;
        changeConstraints = false;
        uniqueConstraints = false;

        addRequirements(templateSubsystem);
    }

    public PositionCommand(TemplateSubsystem templateSubsystem, double goal, boolean shouldNotEnd) {
        this(templateSubsystem, goal);
        this.shouldNotEnd = shouldNotEnd;
    }

    public PositionCommand(TemplateSubsystem templateSubsystem, double goal,
                           double velocity, double acceleration, double jerk) {
        this(templateSubsystem, goal);

        this.velocity = velocity;
        this.acceleration = acceleration;
        this.jerk = jerk;

        changeConstraints = true;
        uniqueConstraints = true;
    }

    /** Also moves another motor group of the same subsystem. */
    public PositionCommand withGoal(MotorGroup motorGroup, double goal) {
        goals.put(motorGroup, goal);
        return this;
    }

    @Override
    public void initialize() {
        applyGoals();
        if (changeConstraints) {
            applyConstraints();
            changeConstraints = false;
        }
        templateSubsystem.setCommandRunning(true);
    }

    @Override
    public void execute() {
        if (updateGoalPosition) {
            applyGoals();
            updateGoalPosition = false;
        }
        if (changeConstraints) {
            applyConstraints();
            changeConstraints = false;
        }
    }

    @Override
    public boolean isFinished() {
        if (shouldNotEnd) return false;
        for (MotorGroup group : goals.keySet()) {
            if (!group.isMechAtGoal(false)) return false;
        }
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        goals.keySet().forEach(group -> group.setFollowLastMechProfile(true));
        templateSubsystem.setCommandRunning(false);
        if (uniqueConstraints) changeConstraints = true;
    }

    /** Changes the goal of the motor group given to the constructor. */
    public void setGoal(double goal) {
        setGoal(motorGroup, goal);
    }

    public void setGoal(MotorGroup motorGroup, double goal) {
        goals.put(motorGroup, goal);
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

    private void applyGoals() {
        goals.forEach((group, goal) -> group.setPosition(goal));
    }

    private void applyConstraints() {
        goals.keySet().forEach(group -> group.setConstraints(velocity, acceleration, jerk));
    }
}
