package frc.robot.subsystems.templates;

import org.wpilib.command2.Command;

public class VelocityCommand extends Command {
    private double goal;
    private double secondaryGoal;

    private TemplateSubsystem templateSubsystem;
    private boolean updateVelocity;

    public VelocityCommand(TemplateSubsystem templateSubsystem, double goal) {
        this.templateSubsystem = templateSubsystem;
        this.goal = goal;
        updateVelocity = false;

        addRequirements(templateSubsystem);
    }

    public VelocityCommand(TemplateSubsystem templateSubsystem, double goal, double secondaryGoal) {
        this.templateSubsystem = templateSubsystem;
        this.goal = goal;
        this.secondaryGoal = secondaryGoal;
        updateVelocity = false;

        addRequirements(templateSubsystem);
    }

    @Override
    public void initialize() {
        templateSubsystem.setVelocity(goal);
        if (templateSubsystem.hasSecondaryMotor())
            templateSubsystem.setSecondaryVelocity(secondaryGoal);
    }

    @Override
    public void execute() {
        if (updateVelocity) {
            templateSubsystem.setVelocity(goal);
            if (templateSubsystem.hasSecondaryMotor())
                templateSubsystem.setSecondaryVelocity(secondaryGoal);
            updateVelocity = false;
        }
    }

    @Override
    public boolean isFinished() {
        // return templateSubsystem.isMechAtGoal(true);
       return false;
    }

    @Override
    public void end(boolean interrupted) {
    }

    public void setGoal(double goal) {
        this.goal = goal;
        updateVelocity = true;
    }

    public void setSecondaryGoal(double secondaryGoal) {
        this.secondaryGoal = secondaryGoal;
        updateVelocity = true;
    }

    public void setBothGoals(double goal, double secondaryGoal) {
        this.goal = goal;
        this.secondaryGoal = secondaryGoal;
        updateVelocity = true;
    }
}
