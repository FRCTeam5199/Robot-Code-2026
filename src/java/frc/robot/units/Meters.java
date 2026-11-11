package frc.robot.units;

import frc.robot.subsystems.templates.LinearSubsystem;

public final class Meters implements LinearUnit {
    public final double magnitude;

    public Meters(double magnitude){
        this.magnitude = magnitude;
    }

    public Meters(MotorRotations rotations, LinearSubsystem subsystem){
        this.magnitude = rotations.magnitude*subsystem.getMetersPerMechRotation()*subsystem.getSpeedGearRatio();
    }

    public MotorRotations asMotorRotations(LinearSubsystem subsystem){
        return new MotorRotations(this.magnitude/subsystem.getMetersPerMechRotation()/subsystem.getSpeedGearRatio());
    }
}
