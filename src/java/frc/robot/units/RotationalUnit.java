package frc.robot.units;

import frc.robot.subsystems.templates.TemplateSubsystem;

public sealed interface RotationalUnit permits MechDegrees, MotorRotations{
    public abstract MotorRotations asMotorRotations(TemplateSubsystem subsystem);
}
