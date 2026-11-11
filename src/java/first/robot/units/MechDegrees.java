package first.robot.units;

import first.robot.subsystems.templates.TemplateSubsystem;

public final class MechDegrees implements RotationalUnit {
    public final double magnitude;

    public MechDegrees(double magnitude){
        this.magnitude = magnitude;
    }

    public MechDegrees(MotorRotations rotations, TemplateSubsystem subsystem){
        magnitude = rotations.magnitude*360*subsystem.getSpeedGearRatio();
    }

    public MotorRotations asMotorRotations(TemplateSubsystem subsystem){
        return new MotorRotations(magnitude*360/subsystem.getSpeedGearRatio());
    }
}
