package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
import frc.robot.utility.LimelightHelpers;

public class Vision extends SubsystemBase {
    public static CommandSwerveDrivetrain commandSwerveDrivetrain = RobotContainer.commandSwerveDrivetrain;
    public volatile static LimelightHelpers.PoseEstimate limelightRightData;
    public volatile static LimelightHelpers.PoseEstimate limelightLeftData;
    public volatile static LimelightHelpers.PoseEstimate limelightTurretData;
    public static Timer originTimer = new Timer();
    private static Vision vision;
    private final TimeInterpolatableBuffer<Rotation3d> turretAngleBuffer =
            TimeInterpolatableBuffer.createBuffer(Constants.TURRET_BUFFER_SIZE);

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
        if (LimelightHelpers.getTV(Constants.LIMELIGHT_LEFT_NAME)) {
            LimelightHelpers.SetRobotOrientation(Constants.LIMELIGHT_LEFT_NAME,
                    commandSwerveDrivetrain.getPigeon2().getYaw().getValueAsDouble(), 0, 0, 0, 0, 0);
            limelightLeftData = LimelightHelpers
                    .getBotPoseEstimate_wpiBlue_MegaTag2(Constants.LIMELIGHT_LEFT_NAME);

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
                        .getDistance(limelightLeftData.pose.getTranslation()) < 2d
                        || originTimer.get() < 5) {

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
                        .getDistance(limelightRightData.pose.getTranslation()) < 2d
                        || originTimer.get() < 5) {

                    commandSwerveDrivetrain.addVisionMeasurement(limelightRightData.pose,
                            limelightRightData.timestampSeconds, VecBuilder
                                    .fill(xyStdev, xyStdev, 9999999999d));
                }
            }
        }
    }

    public Pose3d getTurretCameraPose(double timeStamp) {
        if (turretAngleBuffer.getSample(timeStamp).isEmpty())
            return new Pose3d(Translation3d.kZero, Rotation3d.kZero);
        return Constants.ROBOT_RELATIVE_TURRET_POSE.transformBy(
                        new Transform3d(Translation3d.kZero,
                                turretAngleBuffer.getSample(timeStamp).get()))
                .transformBy(Constants.TURRET_TO_CAMERA);
    }

    public void addSample(double degrees) {
        turretAngleBuffer.addSample(Timer.getTimestamp(),
                new Rotation3d(0, 0, Math.toRadians(-degrees)));
    }

    @Override
    public void periodic() {
//        if (LimelightHelpers.getTV(Constants.LIMELIGHT_TURRET_NAME)) {
//            LimelightHelpers.SetRobotOrientation(Constants.LIMELIGHT_TURRET_NAME,
//                    commandSwerveDrivetrain.getPigeon2().getYaw().getValueAsDouble(),
//                    0, 0, 0, 0, 0);
//
//            limelightTurretData = LimelightHelpers
//                    .getBotPoseEstimate_wpiBlue_MegaTag2(Constants.LIMELIGHT_TURRET_NAME);
//            if (limelightTurretData != null) {
////                System.out.println("Limelight Timestamp: " + limelightTurretData.timestampSeconds);
////                System.out.println("Timer Timestamp: " + Timer.getFPGATimestamp());
//
//                Pose3d turretLimelightPose = getTurretCameraPose(limelightTurretData.timestampSeconds);
//
//                LimelightHelpers.setCameraPose_RobotSpace(Constants.LIMELIGHT_TURRET_NAME,
//                        turretLimelightPose.getX(), turretLimelightPose.getY(), turretLimelightPose.getZ(),
//                        Math.toDegrees(turretLimelightPose.getRotation().getX()),
//                        Math.toDegrees(turretLimelightPose.getRotation().getY()),
//                        -Math.toDegrees(turretLimelightPose.getRotation().getZ()));
//
//                limelightTurretData = LimelightHelpers
//                        .getBotPoseEstimate_wpiBlue_MegaTag2(Constants.LIMELIGHT_TURRET_NAME);
//
//                double xyStdev = .3;
////
//                if (limelightTurretData.tagCount < 2) {
//                    xyStdev *= Math.pow(limelightTurretData.avgTagDist, 3);
//                } else {
//                    xyStdev *= limelightTurretData.avgTagDist;
//                }
//
////                if (RobotContainer.getPose().getTranslation()
////                        .getDistance(new Translation2d(0, 0)) < .25)
////                    originTimer.restart();
////
//                //Only adds pose when it is less than 1m different from our current location
//                //or when we're at the origin (haven't gotten vision data yet)
////                if (RobotContainer.getPose().getTranslation()
////                        .getDistance(limelightTurretData.pose.getTranslation()) < 1d
////                        || originTimer.get() < 5) {
////
//                commandSwerveDrivetrain.addVisionMeasurement(limelightTurretData.pose,
//                        limelightTurretData.timestampSeconds, VecBuilder
//                                .fill(xyStdev, xyStdev, 9999999999d));
////                }
//            }
//        }
    }
}
