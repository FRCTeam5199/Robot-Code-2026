package frc.robot.subsystems.templates;

import java.util.LinkedHashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * Runs motor groups at set velocities. A command requires its whole subsystem, so to run several
 * groups of one subsystem together, use {@link #withGoal} instead of two separate commands:
 *
 * <pre>{@code
 * new VelocityCommand(indexer, indexer.getUpper(), 20).withGoal(indexer.getLower(), 60);
 * }</pre>
 */
public class VelocityCommand extends Command {
    private final MotorGroup motorGroup;
    private final Map<MotorGroup, Double> goals = new LinkedHashMap<>();
    private boolean updateVelocity;

    /** Runs the subsystem's first motor group. */
    public VelocityCommand(TemplateSubsystem templateSubsystem, double goal) {
        this(templateSubsystem, templateSubsystem.getMotorGroup(), goal);
    }

    /** Runs a specific motor group of the subsystem. */
    public VelocityCommand(TemplateSubsystem templateSubsystem, MotorGroup motorGroup, double goal) {
        this.motorGroup = motorGroup;
        goals.put(motorGroup, goal);
        updateVelocity = false;

        addRequirements(templateSubsystem);
    }

    /** Also runs another motor group of the same subsystem. */
    public VelocityCommand withGoal(MotorGroup motorGroup, double goal) {
        goals.put(motorGroup, goal);
        return this;
    }

    @Override
    public void initialize() {
        applyGoals();
    }

    @Override
    public void execute() {
        if (updateVelocity) {
            applyGoals();
            updateVelocity = false;
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
    }

    /** Changes the goal of the motor group given to the constructor. */
    public void setGoal(double goal) {
        setGoal(motorGroup, goal);
    }

    public void setGoal(MotorGroup motorGroup, double goal) {
        goals.put(motorGroup, goal);
        updateVelocity = true;
    }

    private void applyGoals() {
        goals.forEach((group, goal) -> group.setVelocity(goal));
    }
}
