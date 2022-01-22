package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.teleop.TeleOP;

import java.util.LinkedHashMap;

public class SequentialMovements {
    public LinkedHashMap<GoToPosition, Boolean> waypoints;
    public int threshold;
    public TeleOP teleOP;

    public SequentialMovements(LinkedHashMap<GoToPosition, Boolean> waypoints, int threshold,TeleOP teleOp){
        this.waypoints = waypoints;
        this.threshold = threshold;
        this.teleOP = teleOp;
    }

    public boolean runMovements() {
        GoToPosition currentWaypoint = null;
        for(GoToPosition waypoint : waypoints.keySet()){
            if(!waypoints.get(waypoint)){
                currentWaypoint = waypoint;
                teleOP.telemetry.addData("Current Waypoint:",  currentWaypoint.targetPosition.x);
                break;
            }
        }

        if(currentWaypoint == null){
            teleOP.telemetry.addLine("Sequential Movements finished");
            return true;
        }

        boolean finished = currentWaypoint.runWithPID(threshold);

        if(!finished){
            waypoints.remove(currentWaypoint);
        }

        return false;

    }
}
