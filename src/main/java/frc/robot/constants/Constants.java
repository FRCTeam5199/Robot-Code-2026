package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Constants {
    public static final int XBOX_PORT = 0;
    public static final Translation2d RED_HUB_CENTER = new Translation2d(11.919, 4.041); //fix later
    public static final Translation2d RED_HUB_FRONT_CENTER = new Translation2d(12.57, 4.059); //fix later
    public static final Transform2d ROBOT_TO_TURRET = new Transform2d(-.13335, -.13335, new Rotation2d(0));
}