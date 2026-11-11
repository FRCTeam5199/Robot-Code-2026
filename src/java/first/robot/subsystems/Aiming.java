
package first.robot.subsystems;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.stream.Stream;

import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchState;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.interpolation.InterpolatingDoubleTreeMap;
import first.robot.RobotContainer;
import first.robot.constants.MeasurementConstants;
import first.robot.constants.SetPoints;
import first.robot.units.MechDegrees;
import first.robot.units.MotorRotations;

public class Aiming {

    private enum ShootingState{
        SHOOTING,
        LEFT_SHUTTLE,
        RIGHT_SHUTTLE
    }

    private final RobotContainer robot;

    private final Pose2d HUB_POSE;

    private final Transform2d TURRET_OFFSET;

    private final double centerX = 13;
    private final double allianceX = 19;

    private SetPoints currentPoint = SetPoints.POINT0; // default / reset
    private boolean updateCurrentPoint = true;

    private boolean autoAimEnabled = false;

    private final Transform2d[] corners;

    private final InterpolatingDoubleTreeMap hoodPositionDoubleTreeMap = new InterpolatingDoubleTreeMap();    // in mech degrees
    private final InterpolatingDoubleTreeMap shooterVelocityDoubleTreeMap = new InterpolatingDoubleTreeMap(); // in motor rotations per second

    // time it takes for the ball to reach the hub
    private final double shotTime = 0.3;

    public Aiming(RobotContainer robot){
        this.robot = robot;

        HUB_POSE = new Pose2d(12,84, new Rotation2d(0));

        TURRET_OFFSET = new Transform2d(new Translation2d(-0.13335, 0.13335), new Rotation2d(0));

        corners = new Transform2d[]{
            MeasurementConstants.BACK_LEFT_CORNER,
            MeasurementConstants.FRONT_LEFT_CORNER,
            MeasurementConstants.BACK_RIGHT_CORNER,
            MeasurementConstants.FRONT_RIGHT_CORNER
        };

        // filler data:
        for(double i = 0; i < 5; i++){ // dist, hood pos in mech degrees
            hoodPositionDoubleTreeMap.put(i*2, i*i);
        }

        // filler data:
        for(double i = 0; i < 7; i++){ // dist, shooter vel in motor rots / second
            shooterVelocityDoubleTreeMap.put(i*2, i*i*103);
        }
    }

    public void setAutoAimEnabled(boolean enabled){
        autoAimEnabled = enabled;
    }

    public void toggleAutoAim(){
        autoAimEnabled = !autoAimEnabled;
    }

    public void setPoint(SetPoints point){
        this.currentPoint = point;
        updateCurrentPoint = true;
        autoAimEnabled = false;
    }

    public void setAutoAimTarget(){

        Pose2d currentPose = robot.getPose();

        // start with the current robot position
        Pose2d predictedPose = currentPose;

        double target_turret_rotation = 0;
        double dist = 0;

        // look ahead, do 5 iterations to collapse the error
        for(int i = 0; i < 5; i++){

            Pose2d turretPose = predictedPose.transformBy(TURRET_OFFSET);

            target_turret_rotation = turretPose.getTranslation()
                .minus(HUB_POSE.getTranslation())
                .getAngle()
                .orElseThrow()
                .getDegrees() - turretPose.getRotation().getDegrees();

            dist = turretPose.getTranslation().getDistance(HUB_POSE.getTranslation());


            predictedPose = currentPose;
        }

        robot.updateTurretCommandPositions(
            new MechDegrees(target_turret_rotation),
            new MechDegrees(hoodPositionDoubleTreeMap.get(dist)),
            new MotorRotations(shooterVelocityDoubleTreeMap.get(dist))
        );
    }

    public Pose2d getAbsolutePose2d(Transform2d relativeOffset){
        return robot.getPose().transformBy(relativeOffset);
    }

    public ShootingState getShootingState(){

        // on red alliance, x > alliance zone x is shooting, on blue x < alliance zone x in shooting
        boolean isBlueOrNull = MatchState.getAlliance().map((a) -> a.equals(Alliance.BLUE)).orElse(true);

        if(Arrays.stream(corners).anyMatch((corner) -> (getAbsolutePose2d(corner).getX() < allianceX) == isBlueOrNull)){
            return ShootingState.SHOOTING;
        }

        // shuttling (USE Y x is forward y is left/right)
        // on blue alliance, y > center Y is left, y < center Y is right
        // on red alliance, y > center Y is right, y < center Y is left
        return (isBlueOrNull == (robot.getPose().getY() > centerX)) ? ShootingState.LEFT_SHUTTLE : ShootingState.RIGHT_SHUTTLE;
    }

    public void update(){
        if (autoAimEnabled){
            setAutoAimTarget();
        } else if (updateCurrentPoint){
            robot.updateTurretCommandPositions(
                currentPoint.turretPosition,
                currentPoint.hoodPosition,
                currentPoint.shooterVelocity
            );

            updateCurrentPoint = false;
        }
    }
}
