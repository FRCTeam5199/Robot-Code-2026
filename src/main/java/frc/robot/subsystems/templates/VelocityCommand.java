package frc.robot.subsystems.templates;

import java.util.LinkedHashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * Runs motor groups at set velocities. A command requires its whole subsystem, so to give groups of
 * one subsystem different goals, use {@link #withGoal} instead of two separate commands:
 *
 * <pre>{@code
 * new VelocityCommand(indexer, indexer.getUpper(), 20).withGoal(indexer.getLower(), 60);
 * }</pre>
 */
public class VelocityCommand extends Command {
    private final Map<MotorGroup, Double> goals = new LinkedHashMap<>();
    private boolean updateVelocity;

    /** Runs every motor group of the subsystem at the same velocity. */
    public VelocityCommand(TemplateSubsystem templateSubsystem, double goal) {
        for (MotorGroup motorGroup : templateSubsystem.getMotorGroups()) {
            goals.put(motorGroup, goal);
        }

        addRequirements(templateSubsystem);
    }

    /** Runs one motor group of the subsystem. */
    public VelocityCommand(TemplateSubsystem templateSubsystem, MotorGroup motorGroup, double goal) {
        goals.put(motorGroup, goal);

        addRequirements(templateSubsystem);
    }

    /** Sets the goal of one motor group of the same subsystem, adding it to this command if needed. */
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

    /** Changes the goal of every motor group in this command. */
    public void setGoal(double goal) {
        goals.replaceAll((motorGroup, oldGoal) -> goal);
        updateVelocity = true;
    }

    public void setGoal(MotorGroup motorGroup, double goal) {
        goals.put(motorGroup, goal);
        updateVelocity = true;
    }

    private void applyGoals() {
        goals.forEach((group, goal) -> group.setVelocity(goal));
    }
}
