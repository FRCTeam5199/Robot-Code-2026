package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Constants {
    public static final int XBOX_PORT = 0;
    public static final Translation2d TRENCH = new Translation2d(11.916, 4.035);
    public static final Translation2d RED_HUB_CENTER = new Translation2d(11.916, 4.035);
    public static final Translation2d SHUTTLE_LEFT_CORNER = new Translation2d(16.124, 0.406);
    public static final Translation2d SHUTTLE_RIGHT_CORNER = new Translation2d(16.124, 7.651);
    public static final Translation2d RED_HUB_FRONT_CENTER = new Translation2d(12.513, 4.035);
    public static final Transform2d ROBOT_TO_TURRET = new Transform2d(-.13335, .13335, new Rotation2d(0));
    public static final double PHASE_DELAY = 0;
    public static final double CENTER_TO_BUMPER = 0; //add this later
    public static final Transform2d FRONT_LEFT_CORNER = new Transform2d(-CENTER_TO_BUMPER, -CENTER_TO_BUMPER, new Rotation2d(0));
    public static final Transform2d FRONT_RIGHT_CORNER = new Transform2d(CENTER_TO_BUMPER, -CENTER_TO_BUMPER, new Rotation2d(0));
    public static final Transform2d BACK_LEFT_CORNER = new Transform2d(-CENTER_TO_BUMPER, CENTER_TO_BUMPER, new Rotation2d(0));
    public static final Transform2d BACK_RIGHT_CORNER = new Transform2d(CENTER_TO_BUMPER, CENTER_TO_BUMPER, new Rotation2d(0));
}