package frc.robot.subsystems.templates;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Base class for a mechanism made of one or more {@link MotorGroup}s. Subclasses create their groups
 * in their constructor, and a group can have any number of followers:
 *
 * <pre>{@code
 * super("Indexer");
 *
 * upper = addMotorGroup(new MotorGroup(...));
 * upper.configureMotor(...);
 * upper.addFollower(IndexerConstants.SECOND_UPPER_MOTOR_ID, IndexerConstants.SECOND_UPPER_INVERTED);
 *
 * lower = addMotorGroup(new MotorGroup(...));
 * lower.configureMotor(...);
 * }</pre>
 */
public class TemplateSubsystem extends SubsystemBase {
    private final List<MotorGroup> motorGroups = new ArrayList<>();
    private boolean isCommandRunning = false;
    private String name;

    public TemplateSubsystem(String subsystemName) {
        this.name = subsystemName;
    }

    public String getName() {
        return name;
    }

    /**
     * @return the group that was passed in, so it can be stored and configured in one line
     */
    protected MotorGroup addMotorGroup(MotorGroup motorGroup) {
        motorGroups.add(motorGroup);
        return motorGroup;
    }

    /**
     * @return the first group added, for subsystems with one group. Subsystems with several groups
     * should have a getter for each one, like IntakeLinearSubsystem's getLeft() and getRight().
     */
    public MotorGroup getMotorGroup() {
        return motorGroups.get(0);
    }

    public List<MotorGroup> getMotorGroups() {
        return Collections.unmodifiableList(motorGroups);
    }

    public void setBrake(boolean brake) {
        motorGroups.forEach(motorGroup -> motorGroup.setBrake(brake));
    }

    @Override
    public void periodic() {
        motorGroups.forEach(MotorGroup::periodic);
    }

    public boolean isCommandRunning() {
        return isCommandRunning;
    }

    public void setCommandRunning(boolean commandRunning) {
        isCommandRunning = commandRunning;
    }
}
