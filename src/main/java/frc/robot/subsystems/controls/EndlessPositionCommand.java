package frc.robot.subsystems.controls;

import frc.robot.subsystems.templates.TemplateSubsystem;
import frc.robot.units.RotationalUnit;

public class EndlessPositionCommand extends TemplatePositionCommand {
    public EndlessPositionCommand(TemplateSubsystem subsystem, RotationalUnit position){
        super(subsystem, position);
    }
    
    @Override
    public boolean isFinished(){
        return false;
    }
}
