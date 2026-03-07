package frc.robot.utility;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public enum ClimberSetpoint {
    //15.270, 3.226, 180d
    CLIMB_LEFT_RED_PREP(new Pose2d(15.360, 3.2, new Rotation2d(Math.toRadians(180d)))),
    //    CLIMB_LEFT_RED(new Pose2d(15.360, 3.5, new Rotation2d(Math.toRadians(180d)))),
    CLIMB_LEFT_RED(new Pose2d(15.52, 2.32, new Rotation2d(Math.toRadians(180d)))),
    CLIMB_RIGHT_RED(new Pose2d(15.270, 3.226, new Rotation2d(Math.toRadians(0d))));


    final Pose2d pose2d;

    ClimberSetpoint(Pose2d pose2d) {
        this.pose2d = pose2d;
    }

    public Pose2d getPose2d() {
        return pose2d;
    }
}


