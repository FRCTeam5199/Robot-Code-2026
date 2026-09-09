package frc.robot.utility;

import org.wpilib.math.geometry.Translation2d;
import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.RobotState;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchType;
import org.wpilib.driverstation.DriverStationErrors;
import frc.robot.Robot;

public class AllianceFlipper {
    public static Translation2d getCorrectAlliance(Translation2d blueTranslation,
                                                   Translation2d redTranslation) {
        Alliance alliance = Robot.getAlliance();
        if (alliance == null || alliance.equals(Alliance.RED))
            return redTranslation;
        return blueTranslation;
    }
}
