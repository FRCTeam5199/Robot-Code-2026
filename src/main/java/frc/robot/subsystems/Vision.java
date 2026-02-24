package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotContainer;
import frc.robot.utility.LimelightHelpers;

public class Vision {
    public static CommandSwerveDrivetrain commandSwerveDrivetrain = RobotContainer.commandSwerveDrivetrain;
    public volatile static LimelightHelpers.PoseEstimate limelightRightData;
    public volatile static LimelightHelpers.PoseEstimate limelightLeftData;
    public static Timer originTimer = new Timer();
    private static Vision vision;

    private Vision() {
        LimelightHelpers.setCameraPose_RobotSpace("limelight-left",
                -.316, -.316, .453, 0, 5, 135.218);
        LimelightHelpers.setCameraPose_RobotSpace("limelight-right",
                -.317, .317, .436, 180, 5, -135.218);
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
        if (LimelightHelpers.getTV("limelight-left")) {
            LimelightHelpers.SetRobotOrientation("limelight-left",
                    commandSwerveDrivetrain.getPigeon2().getYaw().getValueAsDouble(), 0, 0, 0, 0, 0);
            limelightLeftData = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-left");

            if (limelightLeftData != null) {
                double xyStdev = .3;

                if (limelightLeftData.tagCount < 2) {
                    xyStdev *= Math.pow(limelightLeftData.avgTagDist, 3);
                } else {
                    xyStdev *= limelightLeftData.avgTagDist;
                }

                if (RobotContainer.getPose().getTranslation()
                        .getDistance(new Translation2d(0, 0)) < .25)
                    originTimer.restart();

                //Only adds pose when it is less than 1m different from our current location
                //or when we're at the origin (haven't gotten vision data yet)
                if (RobotContainer.getPose().getTranslation()
                        .getDistance(limelightLeftData.pose.getTranslation()) < 1d
                        || originTimer.get() < 5) {

                    commandSwerveDrivetrain.addVisionMeasurement(limelightLeftData.pose,
                            limelightLeftData.timestampSeconds, VecBuilder.fill(xyStdev, xyStdev, 9999999999d));
                }
            }
        }

        if (LimelightHelpers.getTV("limelight-right")) {
            LimelightHelpers.SetRobotOrientation("limelight-right",
                    commandSwerveDrivetrain.getPigeon2().getYaw().getValueAsDouble(), 0, 0, 0, 0, 0);
            limelightRightData = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-right");

            if (limelightRightData != null) {
                double xyStdev = .3;

                if (limelightRightData.tagCount < 2) {
                    xyStdev *= Math.pow(limelightRightData.avgTagDist, 3);
                } else {
                    xyStdev *= limelightRightData.avgTagDist;
                }

                if (RobotContainer.getPose().getTranslation()
                        .getDistance(new Translation2d(0, 0)) < .25)
                    originTimer.restart();

                //Only adds pose when it is less than 1m different from our current location
                //or when we're at the origin (haven't gotten vision data yet)
                if (RobotContainer.getPose().getTranslation()
                        .getDistance(limelightRightData.pose.getTranslation()) < 1d
                        || originTimer.get() < 5) {

                    commandSwerveDrivetrain.addVisionMeasurement(limelightRightData.pose,
                            limelightRightData.timestampSeconds, VecBuilder.fill(xyStdev, xyStdev, 9999999999d));
                }
            }
        }
    }
}
