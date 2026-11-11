package first.robot.constants;

import first.robot.units.MechDegrees;
import first.robot.units.MotorRotations;
import first.robot.units.RotationalUnit;

public enum SetPoints {
	POINT0(new MotorRotations(0), new MotorRotations(0), new MotorRotations(0)), // default/reset pos
	POINT1(new MechDegrees(0), new MechDegrees(0), new MotorRotations(0)),
	POINT2(new MechDegrees(0), new MechDegrees(0), new MotorRotations(0)),
	POINT3(new MechDegrees(0), new MechDegrees(0), new MotorRotations(0)),
	POINT4(new MechDegrees(0), new MechDegrees(0), new MotorRotations(0));
	
	
	public final RotationalUnit turretPosition;
	public final RotationalUnit hoodPosition;
	public final RotationalUnit shooterVelocity;

	private SetPoints(RotationalUnit turretPosition, RotationalUnit hoodPosition, RotationalUnit shooterVelocity){
		this.turretPosition = turretPosition;
		this.hoodPosition = hoodPosition;
		this.shooterVelocity = shooterVelocity;
	}
}