package org.firstinspires.ftc.teamcode;

import java.util.LinkedHashMap;

public class SequentialMovements {
    public LinkedHashMap<GoToPosition, Boolean> waypoints;
    public int threshold;

    public SequentialMovements(LinkedHashMap<GoToPosition, Boolean> waypoints, int threshold){
        this.waypoints = waypoints;
        this.threshold = threshold;
    }

    public boolean runMovements() {
        GoToPosition currentWaypoint = null;
        for(GoToPosition waypoint : waypoints.keySet()){
            if(!waypoints.get(waypoint) && currentWaypoint == null){
                currentWaypoint = waypoint;
            }
        }

        if(currentWaypoint == null){
            return true;
        }

        boolean finished = currentWaypoint.runWithPID(threshold);

        if(finished){
            waypoints.put(currentWaypoint,false);
        }

        return false;

    }
}
