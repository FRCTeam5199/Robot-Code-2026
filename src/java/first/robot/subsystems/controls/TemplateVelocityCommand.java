package first.robot.subsystems.controls;

import org.wpilib.command2.Command;
import first.robot.subsystems.templates.TemplateSubsystem;
import first.robot.units.RotationalUnit;

public class TemplateVelocityCommand extends Command implements MovementCommand {
    private final TemplateSubsystem subsystem;
    private final RotationalUnit velocity;

    public TemplateVelocityCommand(TemplateSubsystem subsystem, RotationalUnit velocity){
        this.subsystem = subsystem;
        this.velocity = velocity;
        addRequirements(subsystem);
    }

    @Override
    public void initialize(){
        subsystem.setVelocity(velocity.asMotorRotations(subsystem));
    }

    public boolean isMechAtGoal(){
        return subsystem.isMechAtGoal(false, true);
    }

    @Override
    public boolean isFinished(){
        return isMechAtGoal();
    }

    public void updateVelocity(RotationalUnit goal){
        subsystem.setVelocity(goal.asMotorRotations(subsystem));
    }
}
