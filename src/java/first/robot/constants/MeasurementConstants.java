package first.robot.constants;

import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation2d;

public final class MeasurementConstants {
    public static final double HUB_RADIUS = .52959;
    public static final double MAX_SPEED = TunerConstants.kSpeedAt12Volts.baseUnitMagnitude();
    public static final double MAX_ANGULAR_RATE = 2.5 * Math.PI;
    public static final double TURRET_BUFFER_SIZE = 20d;


    /*
        OFFSETS
    */
    public static final double SHUTTLING_DISTANCE_FROM_WALL = 1.75;

    public static final Translation2d RED_HUB_CENTER = new Translation2d(11.916, 4.035);
    public static final Translation2d RED_HUB_FRONT_CENTER = new Translation2d(12.513, 4.035);
    public static final Translation2d RED_SHUTTLE_LEFT_CORNER = new Translation2d(15.5, SHUTTLING_DISTANCE_FROM_WALL);
    public static final Translation2d RED_SHUTTLE_RIGHT_CORNER = new Translation2d(15.5, 8.069 - SHUTTLING_DISTANCE_FROM_WALL);

    public static final Translation2d BLUE_HUB_CENTER = new Translation2d(4.626, 4.035);
    public static final Translation2d BLUE_HUB_FRONT_CENTER = new Translation2d(4.029, 4.035);
    public static final Translation2d BLUE_SHUTTLE_RIGHT_CORNER = new Translation2d(.740, SHUTTLING_DISTANCE_FROM_WALL);
    public static final Translation2d BLUE_SHUTTLE_LEFT_CORNER = new Translation2d(.740, 8.069 - SHUTTLING_DISTANCE_FROM_WALL);

    /*
        ROBOT OFFSETS
    */

    public static final double CENTER_TO_BUMPER_LONG = .9 / 2d;
    public static final double CENTER_TO_BUMPER_SHORT = .795 / 2d;

    public static final Transform2d ROBOT_TO_TURRET = new Transform2d(-.13335, .13335, new Rotation2d(0));
    
    public static final Transform2d FRONT_LEFT_CORNER = new Transform2d(CENTER_TO_BUMPER_SHORT, CENTER_TO_BUMPER_LONG, new Rotation2d(0));
    public static final Transform2d FRONT_RIGHT_CORNER = new Transform2d(CENTER_TO_BUMPER_SHORT, -CENTER_TO_BUMPER_LONG, new Rotation2d(0));
    public static final Transform2d BACK_LEFT_CORNER = new Transform2d(-CENTER_TO_BUMPER_SHORT, CENTER_TO_BUMPER_LONG, new Rotation2d(0));
    public static final Transform2d BACK_RIGHT_CORNER = new Transform2d(-CENTER_TO_BUMPER_SHORT, -CENTER_TO_BUMPER_LONG, new Rotation2d(0));
    public static final Pose3d ROBOT_RELATIVE_TURRET_POSE = new Pose3d(ROBOT_TO_TURRET.getX(), -ROBOT_TO_TURRET.getY(), .470, Rotation3d.ZERO);
    public static final Transform3d TURRET_TO_CAMERA = new Transform3d(.041, .162, 0, new Rotation3d(Math.toRadians(180d), Math.toRadians(14.69), 0));
    
}
