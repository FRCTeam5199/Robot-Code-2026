package frc.robot.subsystems.warnings;

public class IllegalCurrentLimitWarning extends Warning {
	public IllegalCurrentLimitWarning(String message) {
		super(message + " Use this.suppressWarning(IllegalCurrentLimit) to disable this message for this class.");
	}
}