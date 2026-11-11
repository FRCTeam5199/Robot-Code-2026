package frc.robot.units;

import frc.robot.subsystems.templates.LinearSubsystem;

public sealed interface LinearUnit permits Meters, MotorRotations {
    public abstract MotorRotations asMotorRotations(LinearSubsystem subsystem);
}
