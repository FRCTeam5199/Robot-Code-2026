package frc.robot.utility;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Robot;

public class AllianceFlipper {
    public static Translation2d getCorrectAlliance(Translation2d blueTranslation,
                                                   Translation2d redTranslation) {
        DriverStation.Alliance alliance = Robot.getAlliance();
        if (alliance == null || alliance.equals(DriverStation.Alliance.Red))
            return redTranslation;
        return blueTranslation;
    }
}
