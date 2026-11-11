package frc.robot.subsystems.warnings;

public class UnimplementedWarning extends Warning {
	public UnimplementedWarning(String message) {
		super(message + " Use this.suppressWarning(UnimplementedWarning) to disable this message for this class.");
	}
}