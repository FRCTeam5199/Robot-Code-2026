package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Constants {
    public static final int XBOX_PORT = 0;
    public static final Translation2d RED_HUB_CENTER = new Translation2d(11.916, 4.035);
    public static final Translation2d RED_HUB_FRONT_CENTER = new Translation2d(12.513, 4.035);
    public static final Transform2d ROBOT_TO_TURRET = new Transform2d(-.13335, .13335, new Rotation2d(0));
    public static final double PHASE_DELAY = .03;
}