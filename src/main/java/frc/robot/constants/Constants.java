package frc.robot.constants;

import edu.wpi.first.math.geometry.*;

public class Constants {
    public static final int XBOX_PORT = 0;
    public static final Translation2d TRENCH = new Translation2d(11.916, 4.035);
    public static final Translation2d RED_HUB_CENTER = new Translation2d(11.916, 4.035);
    public static final Translation2d SHUTTLE_LEFT_CORNER = new Translation2d(15, 1.3);
    public static final Translation2d SHUTTLE_RIGHT_CORNER = new Translation2d(16, 7.35);
    public static final Translation2d RED_HUB_FRONT_CENTER = new Translation2d(12.513, 4.035);
    public static final double HUB_RADIUS = .52959;
    public static final Transform2d ROBOT_TO_TURRET = new Transform2d(-.13335, .13335, new Rotation2d(0));
    public static final double PHASE_DELAY = 0.02;
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
    public static final String LIMELIGHT_LEFT_NAME = "limelight-left"; //10.51.99.5:5801
    public static final String LIMELIGHT_RIGHT_NAME = "limelight-right"; //10.51.99.6:5801
    public static final String LIMELIGHT_TURRET_NAME = "limelight-turret"; //10.51.99.7:5801 - NOT SET YET
}