package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
import frc.robot.utility.LimelightHelpers;

import java.awt.geom.Ellipse2D;

public class Vision {
    public static CommandSwerveDrivetrain commandSwerveDrivetrain = RobotContainer.commandSwerveDrivetrain;
    public volatile static LimelightHelpers.PoseEstimate limelightRightData;
    public volatile static LimelightHelpers.PoseEstimate limelightLeftData;
    public volatile static LimelightHelpers.PoseEstimate limelightFrontData;
    private static Vision vision;
    private final TimeInterpolatableBuffer<Rotation3d> turretAngleBuffer =
            TimeInterpolatableBuffer.createBuffer(Constants.TURRET_BUFFER_SIZE);
//    private final LinearFilter poseFilter =
//            LinearFilter.movingAverage((int) (0.1 / 0.02));

    private Vision() {
        //all numbers: 1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32
        //5,8,9,10,11,2,3,4 - hub april tags
        //filters: 1,2,3,4,5,6,8,9,10,11,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32 - removing outpost, tower, and trench
        //filters: 1,2,3,4,5,6,7,8,9,10,11,12,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32 - removing outpost

        LimelightHelpers.setCameraPose_RobotSpace(Constants.LIMELIGHT_LEFT_NAME,
                -.316, -.316, .453, 0, 5, 135.218);
        LimelightHelpers.setCameraPose_RobotSpace(Constants.LIMELIGHT_RIGHT_NAME,
                -.317, .317, .436, 180, 5, -135.218);
        LimelightHelpers.setCameraPose_RobotSpace(Constants.LIMELIGHT_FRONT_NAME,
                -.182, .156, .445, 0, 10, 0); //forward and side could be switched around, yaw might be 180

        startThread();
    }

    public static Vision getInstance() {
        if (vision == null) vision = new Vision();
        return vision;
    }

    public void startThread() {
        Thread visionThread = new Thread(() -> {
            while (!Thread.interrupted()) {
                updatePoses();
                try {
                    Thread.sleep(10);
                } catch (InterruptedException ignored) {

                }
            }
        });
        visionThread.start();
    }

    public void updatePoses() {
        if (LimelightHelpers.getTV(Constants.LIMELIGHT_LEFT_NAME)) {
            LimelightHelpers.SetRobotOrientation(Constants.LIMELIGHT_LEFT_NAME,
                    commandSwerveDrivetrain.getPigeon2().getYaw().getValueAsDouble(), 0, 0, 0, 0, 0);
            limelightLeftData = LimelightHelpers
                    .getBotPoseEstimate_wpiBlue_MegaTag2(Constants.LIMELIGHT_LEFT_NAME);

            if (limelightLeftData != null) {
                double xyStdev = .5;

                if (limelightLeftData.tagCount < 2) {
                    xyStdev *= Math.pow(limelightLeftData.avgTagDist, 3);
                } else {
                    xyStdev *= limelightLeftData.avgTagDist;
                }

//                System.out.println(limelightLeftData.pose.getTranslation().getDistance(RobotContainer.getPose().getTranslation()));

                if (!RobotContainer.isAutonomous() || limelightLeftData.pose.getTranslation()
                        .getDistance(RobotContainer.getPose().getTranslation()) < .25) {
                    if (!limelightLeftData.pose.equals(new Pose2d(0, 0, new Rotation2d(0)))
                            && !RobotContainer.isClimbing()) {
                        commandSwerveDrivetrain.addVisionMeasurement(limelightLeftData.pose,
                                limelightLeftData.timestampSeconds, VecBuilder
                                        .fill(xyStdev, xyStdev, 9999999999d));
                    }
                }
            }
        }

        if (LimelightHelpers.getTV(Constants.LIMELIGHT_RIGHT_NAME)) {
            LimelightHelpers.SetRobotOrientation(Constants.LIMELIGHT_RIGHT_NAME,
                    commandSwerveDrivetrain.getPigeon2().getYaw().getValueAsDouble(), 0, 0, 0, 0, 0);
            limelightRightData = LimelightHelpers
                    .getBotPoseEstimate_wpiBlue_MegaTag2(Constants.LIMELIGHT_RIGHT_NAME);

            if (limelightRightData != null) {
                double xyStdev = .5;

                if (limelightRightData.tagCount < 2) {
                    xyStdev *= Math.pow(limelightRightData.avgTagDist, 3);
                } else {
                    xyStdev *= limelightRightData.avgTagDist;
                }

                if (RobotContainer.isClimbing() && limelightRightData.pose.getTranslation()
                        .getDistance(RobotContainer.getPose().getTranslation()) > .05)
                    return;

                if (!RobotContainer.isAutonomous() || limelightRightData.pose.getTranslation()
                        .getDistance(RobotContainer.getPose().getTranslation()) < .25) {
                    if (!limelightRightData.pose.equals(new Pose2d(0, 0, new Rotation2d(0)))) {
                        commandSwerveDrivetrain.addVisionMeasurement(limelightRightData.pose,
                                limelightRightData.timestampSeconds, VecBuilder
                                        .fill(xyStdev, xyStdev, 9999999999d));
                    }
                }
            }
        }

        if (LimelightHelpers.getTV(Constants.LIMELIGHT_FRONT_NAME)) {
            LimelightHelpers.SetRobotOrientation(Constants.LIMELIGHT_FRONT_NAME,
                    commandSwerveDrivetrain.getPigeon2().getYaw().getValueAsDouble(), 0, 0, 0, 0, 0);
            limelightFrontData = LimelightHelpers
                    .getBotPoseEstimate_wpiBlue_MegaTag2(Constants.LIMELIGHT_FRONT_NAME);

            if (limelightFrontData != null) {
                double xyStdev = .5;

                if (limelightFrontData.tagCount < 2) {
                    xyStdev *= Math.pow(limelightFrontData.avgTagDist, 3);
                } else {
                    xyStdev *= limelightFrontData.avgTagDist;
                }

                if (!RobotContainer.isAutonomous() || limelightFrontData.pose.getTranslation()
                        .getDistance(RobotContainer.getPose().getTranslation()) < .25) {
                    if (!limelightFrontData.pose.equals(new Pose2d(0, 0, new Rotation2d(0)))
                            && !RobotContainer.isClimbing()) {
                        commandSwerveDrivetrain.addVisionMeasurement(limelightFrontData.pose,
                                limelightFrontData.timestampSeconds, VecBuilder
                                        .fill(xyStdev, xyStdev, 9999999999d));
                    }
                }
            }
        }
    }
}
