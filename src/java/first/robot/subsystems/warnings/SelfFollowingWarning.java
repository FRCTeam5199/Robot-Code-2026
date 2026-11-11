package first.robot.subsystems.warnings;

public class SelfFollowingWarning extends Warning {
	public SelfFollowingWarning(String message) {
		super(message + " Use this.suppressWarning(SelfFollowingWarning) to disable this message for this class.");
	}
}