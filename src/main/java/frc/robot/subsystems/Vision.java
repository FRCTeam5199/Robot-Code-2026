package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
import frc.robot.utility.LimelightHelpers;

public class Vision {
    public static CommandSwerveDrivetrain commandSwerveDrivetrain = RobotContainer.commandSwerveDrivetrain;
    public volatile static LimelightHelpers.PoseEstimate limelightRightData;
    public volatile static LimelightHelpers.PoseEstimate limelightLeftData;
    public volatile static LimelightHelpers.PoseEstimate limelightFrontData;
    private static Vision vision;
    private static Timer rightTimer = new Timer();
    private static Timer leftTimer = new Timer();
    private static Timer frontTimer = new Timer();
    private static double minWrongTime = 1d;
    private static double maxWrongDistance = 1.5;
    private static double maxSingleTagDistance = 1.5;
    private static double maxDoubleTagDistance = 4.15;
    private static double stdDev = .35;

    private Vision() {
        //all numbers: 1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32
        //5,8,9,10,11,2,3,4 - hub april tags
        //filters: 1,2,3,4,5,6,8,9,10,11,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32 - removing outpost, tower, and trench
        //filters: 1,2,3,4,5,6,7,8,9,10,11,12,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32 - removing outpost

        LimelightHelpers.setCameraPose_RobotSpace(Constants.LIMELIGHT_LEFT_NAME,
                -0.290653, -0.341453, 0.498656, 0, 5, 134.782385);
        LimelightHelpers.setCameraPose_RobotSpace(Constants.LIMELIGHT_FRONT_NAME,
                -0.049, 0.296, 0.514, 0, 10, 14.106);
        LimelightHelpers.setCameraPose_RobotSpace(Constants.LIMELIGHT_RIGHT_NAME,
                -0.248110, .3182, 0.509, 0, 0, -160d);
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
        if (RobotContainer.getState() == null) return;
        if (LimelightHelpers.getTV(Constants.LIMELIGHT_LEFT_NAME)) {
            LimelightHelpers.SetRobotOrientation(Constants.LIMELIGHT_LEFT_NAME,
                    commandSwerveDrivetrain.getPigeon2().getYaw().getValueAsDouble(), 0, 0, 0, 0, 0);
            limelightLeftData = LimelightHelpers
                    .getBotPoseEstimate_wpiBlue_MegaTag2(Constants.LIMELIGHT_LEFT_NAME);

            if (limelightLeftData != null) {
                double xyStdev = stdDev;

                boolean shouldAddPose = true;
                if (limelightLeftData.pose.getTranslation()
                        .getDistance(RobotContainer.getPose().getTranslation()) > maxWrongDistance) {
                    if (!leftTimer.isRunning()) leftTimer.start();

                    if (!leftTimer.hasElapsed(minWrongTime)) shouldAddPose = false;
                } else {
                    leftTimer.stop();
                    leftTimer.reset();
                }

                if (limelightLeftData.tagCount < 2) {
                    xyStdev *= Math.pow(limelightLeftData.avgTagDist, 3);
                    if (limelightLeftData.avgTagDist > maxSingleTagDistance) shouldAddPose = false;
                } else {
                    xyStdev *= limelightLeftData.avgTagDist;
                    if (limelightLeftData.avgTagDist > maxDoubleTagDistance) shouldAddPose = false;
                }

                if (limelightLeftData.pose.equals(new Pose2d(0, 0, new Rotation2d(0))))
                    shouldAddPose = false;

                if (shouldAddPose) {
                    commandSwerveDrivetrain.addVisionMeasurement(limelightLeftData.pose,
                            limelightLeftData.timestampSeconds, VecBuilder
                                    .fill(xyStdev, xyStdev, 9999999999d));
                }
            }
        }

        if (LimelightHelpers.getTV(Constants.LIMELIGHT_RIGHT_NAME)) {
            LimelightHelpers.SetRobotOrientation(Constants.LIMELIGHT_RIGHT_NAME,
                    commandSwerveDrivetrain.getPigeon2().getYaw().getValueAsDouble(), 0, 0, 0, 0, 0);
            limelightRightData = LimelightHelpers
                    .getBotPoseEstimate_wpiBlue_MegaTag2(Constants.LIMELIGHT_RIGHT_NAME);

            if (limelightRightData != null) {
                double xyStdev = stdDev;

                boolean shouldAddPose = true;
                if (limelightRightData.pose.getTranslation()
                        .getDistance(RobotContainer.getPose().getTranslation()) > maxWrongDistance) {
                    if (!rightTimer.isRunning()) rightTimer.start();

                    if (!rightTimer.hasElapsed(minWrongTime)) shouldAddPose = false;
                } else {
                    rightTimer.stop();
                    rightTimer.reset();
                }

                if (limelightRightData.tagCount < 2) {
                    xyStdev *= Math.pow(limelightRightData.avgTagDist, 3);
                    if (limelightRightData.avgTagDist > maxSingleTagDistance) shouldAddPose = false;
                } else {
                    xyStdev *= limelightRightData.avgTagDist;
                    if (limelightRightData.avgTagDist > maxDoubleTagDistance) shouldAddPose = false;
                }

                if (limelightRightData.pose.equals(new Pose2d(0, 0, new Rotation2d(0))))
                    shouldAddPose = false;

                if (shouldAddPose) {
                    commandSwerveDrivetrain.addVisionMeasurement(limelightRightData.pose,
                            limelightRightData.timestampSeconds, VecBuilder
                                    .fill(xyStdev, xyStdev, 9999999999d));
                }
            }
        }

        if (LimelightHelpers.getTV(Constants.LIMELIGHT_FRONT_NAME)) {
            LimelightHelpers.SetRobotOrientation(Constants.LIMELIGHT_FRONT_NAME,
                    commandSwerveDrivetrain.getPigeon2().getYaw().getValueAsDouble(), 0, 0, 0, 0, 0);
            limelightFrontData = LimelightHelpers
                    .getBotPoseEstimate_wpiBlue_MegaTag2(Constants.LIMELIGHT_FRONT_NAME);

            if (limelightFrontData != null) {
                double xyStdev = stdDev;

                boolean shouldAddPose = true;
                if (RobotContainer.getPose() != null && limelightFrontData.pose.getTranslation()
                        .getDistance(RobotContainer.getPose().getTranslation()) > maxWrongDistance) {
                    if (!frontTimer.isRunning()) frontTimer.start();

                    if (!frontTimer.hasElapsed(minWrongTime)) shouldAddPose = false;
                } else {
                    frontTimer.stop();
                    frontTimer.reset();
                }

                if (limelightFrontData.tagCount < 2) {
                    xyStdev *= Math.pow(limelightFrontData.avgTagDist, 3);
                    if (limelightFrontData.avgTagDist > maxSingleTagDistance) shouldAddPose = false;
                } else {
                    xyStdev *= limelightFrontData.avgTagDist;
                    if (limelightFrontData.avgTagDist > maxDoubleTagDistance) shouldAddPose = false;
                }

                if (limelightFrontData.pose.equals(new Pose2d(0, 0, new Rotation2d(0))))
                    shouldAddPose = false;

                if (shouldAddPose) {
                    commandSwerveDrivetrain.addVisionMeasurement(limelightFrontData.pose,
                            limelightFrontData.timestampSeconds, VecBuilder
                                    .fill(xyStdev, xyStdev, 9999999999d));
                }
            }
        }
    }
}
