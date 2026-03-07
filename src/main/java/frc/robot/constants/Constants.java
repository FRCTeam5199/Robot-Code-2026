package frc.robot.constants;

import edu.wpi.first.math.geometry.*;

public class Constants {
    public static final int XBOX_PORT = 0;

    public static final Translation2d RED_HUB_CENTER = new Translation2d(11.916, 4.035);
    public static final Translation2d RED_HUB_FRONT_CENTER = new Translation2d(12.513, 4.035);
    public static final Translation2d RED_SHUTTLE_LEFT_CORNER = new Translation2d(15.809, 1.311);
    public static final Translation2d RED_SHUTTLE_RIGHT_CORNER = new Translation2d(15.809, 6.797);

    public static final Translation2d BLUE_HUB_CENTER = new Translation2d(4.029, 4.035);
    public static final Translation2d BLUE_HUB_FRONT_CENTER = new Translation2d(4.626, 4.035);
    public static final Translation2d BLUE_SHUTTLE_LEFT_CORNER = new Translation2d(.740, 1.311);
    public static final Translation2d BLUE_SHUTTLE_RIGHT_CORNER = new Translation2d(.740, 6.797);

    public static final double HUB_RADIUS = .52959;
    public static final Transform2d ROBOT_TO_TURRET = new Transform2d(-.13335, .13335, new Rotation2d(0));
    public static final double PHASE_DELAY = 0.1;
    public static final double ROTATIONAL_PHASE_DELAY = 0.1;
    public static final double CENTER_TO_BUMPER = 0; //add this later
    public static final Transform2d FRONT_LEFT_CORNER = new Transform2d(-CENTER_TO_BUMPER, -CENTER_TO_BUMPER, new Rotation2d(0));
    public static final Transform2d FRONT_RIGHT_CORNER = new Transform2d(CENTER_TO_BUMPER, -CENTER_TO_BUMPER, new Rotation2d(0));
    public static final Transform2d BACK_LEFT_CORNER = new Transform2d(-CENTER_TO_BUMPER, CENTER_TO_BUMPER, new Rotation2d(0));
    public static final Transform2d BACK_RIGHT_CORNER = new Transform2d(CENTER_TO_BUMPER, CENTER_TO_BUMPER, new Rotation2d(0));
    public static final double MAX_SPEED = TunerConstants.kSpeedAt12Volts.baseUnitMagnitude();
    public static final double MAX_ANGULAR_RATE = 2.5 * Math.PI;
    public static final Pose3d ROBOT_RELATIVE_TURRET_POSE = new Pose3d(ROBOT_TO_TURRET.getX(), -ROBOT_TO_TURRET.getY(), .470, Rotation3d.kZero);
    public static final Transform3d TURRET_TO_CAMERA = new Transform3d(.041, .162, 0, new Rotation3d(Math.toRadians(180d), Math.toRadians(14.69), 0));
    public static final double TURRET_BUFFER_SIZE = 20d;
    public static final String LIMELIGHT_LEFT_NAME = "limelight-left"; //10.51.99.11:5801
    public static final String LIMELIGHT_RIGHT_NAME = "limelight-right"; //10.51.99.12:5801
    public static final String LIMELIGHT_FRONT_NAME = "limelight-turret"; //10.51.99.13:5801 - NOT SET YET
}