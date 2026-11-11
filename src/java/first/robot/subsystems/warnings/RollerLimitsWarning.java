package first.robot.subsystems.warnings;

public class RollerLimitsWarning extends Warning {
	public RollerLimitsWarning(String message) {
		super(message + " Use this.suppressWarning(RollerLimitsWarning) to disable this message for this class.");
	}
}