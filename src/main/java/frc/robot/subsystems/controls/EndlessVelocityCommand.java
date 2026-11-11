package frc.robot.subsystems.controls;

import frc.robot.subsystems.templates.TemplateSubsystem;
import frc.robot.units.RotationalUnit;

public class EndlessVelocityCommand extends TemplateVelocityCommand {
    public EndlessVelocityCommand(TemplateSubsystem subsystem, RotationalUnit velocity){
        super(subsystem, velocity);
    }
    
    @Override
    public boolean isFinished(){
        return false;
    }
}
