package first.robot.subsystems.controls;

import org.wpilib.command2.Command;
import first.robot.subsystems.templates.TemplateSubsystem;
import first.robot.units.RotationalUnit;

public class TemplatePositionCommand extends Command implements MovementCommand {
    private final TemplateSubsystem subsystem;
    private RotationalUnit position;

    public TemplatePositionCommand(TemplateSubsystem subsystem, RotationalUnit position){
        this.subsystem = subsystem;
        this.position = position;
        addRequirements(subsystem);
    }

    @Override
    public void initialize(){
        subsystem.setPosition(position.asMotorRotations(subsystem));
    }

    public boolean isMechAtGoal(){
        return subsystem.isMechAtGoal(true, false);
    }

    @Override
    public boolean isFinished(){
        return isMechAtGoal();
    }

    // might need to put in execute to avoid weird stuff (boolean flag)
    public void updatePosition(RotationalUnit position){
        this.position = position;
        subsystem.setPosition(position.asMotorRotations(subsystem));
    }
}
