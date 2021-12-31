package org.firstinspires.ftc.teamcode;

import java.util.TreeMap;

public class SequentialMovements {
    public TreeMap<GoToPosition, Boolean> waypoints;
    public int threshold;


    public SequentialMovements(TreeMap<GoToPosition, Boolean> waypoints, int threshold){
        this.waypoints = waypoints;
        this.threshold = threshold;
    }

    public boolean runMovements(){
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
        if(finished == true){
            waypoints.put(currentWaypoint,false);
        }
        return false;

    }
}
