package first.robot.subsystems.controls;

import first.robot.subsystems.templates.TemplateSubsystem;
import first.robot.units.RotationalUnit;

public class EndlessPositionCommand extends TemplatePositionCommand {
    public EndlessPositionCommand(TemplateSubsystem subsystem, RotationalUnit position){
        super(subsystem, position);
    }
    
    @Override
    public boolean isFinished(){
        return false;
    }
}
