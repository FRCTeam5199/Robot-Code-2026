package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;

public final class Autos {
    private PathPlannerAuto bottomDelayedBump;

    public PathPlannerAuto getLeftDelayedBump() {
        return bottomDelayedBump;
    }

    public PathPlannerAuto getRightDelayedBump() {
        return topDelayedBump;
    }

    public PathPlannerAuto getLeftDoubleTrench() {
        return bottomDoubleTrench;
    }

    public PathPlannerAuto getRightDoubleTrench() {
        return topDoubleTrench;
    }

    public PathPlannerAuto getLeftDelayedTrench() {
        return bottomDelayedTrench;
    }

    public PathPlannerAuto getRightDelayedTrench() {
        return topDelayedTrench;
    }

    private PathPlannerAuto topDelayedBump;
    private PathPlannerAuto bottomDoubleTrench;
    private PathPlannerAuto topDoubleTrench;
    private PathPlannerAuto bottomDelayedTrench;
    private PathPlannerAuto topDelayedTrench;
    private static Autos autos;

    public void initializeAutos() {
        bottomDoubleTrench = new PathPlannerAuto("Red Bottom Double Trench");
        topDoubleTrench = new PathPlannerAuto("Red Top Double Trench");

        bottomDelayedBump = new PathPlannerAuto("Red Bottom Delayed Bump");
        topDelayedBump = new PathPlannerAuto("Red Top Delayed Bump");

        bottomDelayedTrench = new PathPlannerAuto("Red Bottom Delayed Trench");
        topDelayedTrench = new PathPlannerAuto("Red Top Delayed Trench");
    }

    public static Autos getInstance() {
        if (autos == null) autos = new Autos();
        return autos;
    }
}
