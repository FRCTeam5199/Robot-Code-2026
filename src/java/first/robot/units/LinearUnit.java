package first.robot.units;

import first.robot.subsystems.templates.LinearSubsystem;

public sealed interface LinearUnit permits Meters, MotorRotations {
    public abstract MotorRotations asMotorRotations(LinearSubsystem subsystem);
}
