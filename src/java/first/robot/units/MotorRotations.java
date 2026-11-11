package first.robot.units;

import first.robot.subsystems.templates.LinearSubsystem;
import first.robot.subsystems.templates.TemplateSubsystem;

public final class MotorRotations implements LinearUnit, RotationalUnit {
    public final double magnitude;

    public MotorRotations(double magnitude){
        this.magnitude = magnitude;
    }

    public MotorRotations asMotorRotations(LinearSubsystem linearSubsystem){
        return this;
    }

    public MotorRotations asMotorRotations(TemplateSubsystem subsystem){
        return this;
    }
}
