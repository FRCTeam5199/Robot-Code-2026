package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Constants {
    public static final int XBOX_PORT = 0;
    public static final Translation2d RED_HUB = new Translation2d(11.919, 4.041);
    public static final Transform2d ROBOT_TO_TURRET = new Transform2d(-.13335, .13335, new Rotation2d(0));
}