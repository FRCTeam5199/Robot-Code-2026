package first.robot.subsystems.controls;

import first.robot.subsystems.templates.TemplateSubsystem;
import first.robot.units.RotationalUnit;

public class EndlessVelocityCommand extends TemplateVelocityCommand {
    public EndlessVelocityCommand(TemplateSubsystem subsystem, RotationalUnit velocity){
        super(subsystem, velocity);
    }
    
    @Override
    public boolean isFinished(){
        return false;
    }
}
