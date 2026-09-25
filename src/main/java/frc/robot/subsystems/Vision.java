package frc.robot.subsystems;

import com.limelightvision.Limelight;
import com.limelightvision.PoseEstimate;
import com.limelightvision.PoseEstimateType;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.system.Timer;

import java.util.concurrent.ConcurrentLinkedQueue;

public class Vision {
    public static CommandSwerveDrivetrain commandSwerveDrivetrain = RobotContainer.commandSwerveDrivetrain;
    private static Vision vision;
    private static Timer rightTimer;
    private static Timer leftTimer;
    private static Timer frontTimer;

    private static Limelight leftLimelight;
    private static Limelight frontLimelight;
    private static Limelight rightLimelight;

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
        rightTimer = new Timer();
        leftTimer = new Timer();
        frontTimer = new Timer();

        leftLimelight = new Limelight(Constants.LIMELIGHT_LEFT_NAME, new Pose3d(-0.290653, -0.341453, 0.498656, new Rotation3d(0, 5, 134.782385)));
        frontLimelight = new Limelight(Constants.LIMELIGHT_FRONT_NAME, new Pose3d(-0.049, 0.296, 0.514, new Rotation3d(0, 10, 14.106)));
        rightLimelight = new Limelight(Constants.LIMELIGHT_RIGHT_NAME, new Pose3d(-0.248110, .3182, 0.509, new Rotation3d(0, 0, -160d)));
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

        Limelight.setSharedRobotOrientation(commandSwerveDrivetrain.getPigeon2().getYaw().getValueAsDouble());

        updateCameraPose(leftLimelight, leftTimer);
        updateCameraPose(frontLimelight, rightTimer);
        updateCameraPose(rightLimelight, frontTimer);
    }

    public static final ConcurrentLinkedQueue<VisionMeasurement> pendingMeasurements = new ConcurrentLinkedQueue<>();

    public record VisionMeasurement(Pose2d pose, double timestampSeconds, double xyStdev) {
    }

    private void updateCameraPose(Limelight limelight, Timer wrongPoseTimer) {
        if (!limelight.hasTarget()) return;

        PoseEstimate[] allPoses = limelight.readAcceptedPoseEstimates(PoseEstimateType.MT2_WPIBLUE);

        if (allPoses == null) return;

        for (PoseEstimate data : allPoses) {
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

            double xyStdev = .01 * Math.pow(data.avgTagDistanceMeters, 2) / Math.pow(data.fieldedTagCount, 2);

            if (data.pose.equals(new Pose2d(0, 0, new Rotation2d(0))))
                shouldAddPose = false;

            if (shouldAddPose) {
                pendingMeasurements.add(new VisionMeasurement(data.pose, data.timestampSeconds, xyStdev));
            }
        }
    }

    public Limelight getLeftLimelight() {
        return leftLimelight;
    }

    public Limelight getFrontLimelight() {
        return frontLimelight;
    }

    public Limelight getRightLimelight() {
        return rightLimelight;
    }
}