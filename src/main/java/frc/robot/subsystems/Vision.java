package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
import frc.robot.utility.LimelightHelpers;

import java.util.concurrent.ConcurrentLinkedQueue;

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
    private static double maxWrongDistance = .25;
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

        updateCameraPose(Constants.LIMELIGHT_LEFT_NAME, leftTimer);
        updateCameraPose(Constants.LIMELIGHT_RIGHT_NAME, rightTimer);
        updateCameraPose(Constants.LIMELIGHT_FRONT_NAME, frontTimer);
    }

    public static final ConcurrentLinkedQueue<VisionMeasurement> pendingMeasurements = new ConcurrentLinkedQueue<>();

    public record VisionMeasurement(Pose2d pose, double timestampSeconds, double xyStdev) {}

    private void updateCameraPose(String limelightName, Timer wrongPoseTimer) {
        if (!LimelightHelpers.getTV(limelightName)) return;

        LimelightHelpers.SetRobotOrientation(limelightName,
                commandSwerveDrivetrain.getPigeon2().getYaw().getValueAsDouble(), 0, 0, 0, 0, 0);

        LimelightHelpers.PoseEstimate data = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
        if (data == null) return;

        boolean shouldAddPose = true;

        Pose2d currentPose = RobotContainer.getPose();
        if (currentPose != null && data.pose.getTranslation()
                .getDistance(currentPose.getTranslation()) > maxWrongDistance) {
            if (!wrongPoseTimer.isRunning()) wrongPoseTimer.start();
            if (!wrongPoseTimer.hasElapsed(minWrongTime)) shouldAddPose = false;
        } else {
            wrongPoseTimer.stop();
            wrongPoseTimer.reset();
        }

//        if (data.tagCount < 2) {
//            xyStdev *= Math.pow(data.avgTagDist, 3);
//        } else {
//            xyStdev *= data.avgTagDist;
//        }

        double xyStdev = .01 * Math.pow(data.avgTagDist, 2) / Math.pow(data.tagCount, 2);

        if (data.pose.equals(new Pose2d(0, 0, new Rotation2d(0))))
            shouldAddPose = false;

        if (shouldAddPose) {
            pendingMeasurements.add(new VisionMeasurement(data.pose, data.timestampSeconds, xyStdev));
        }
    }
}
