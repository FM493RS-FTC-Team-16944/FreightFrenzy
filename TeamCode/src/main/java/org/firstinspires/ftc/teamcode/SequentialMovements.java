package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.LinkedHashMap;

public class SequentialMovements {
    public LinkedHashMap<GoToPosition, Boolean> waypoints;
    public int threshold;
    public LinearOpMode teleOP;

    public SequentialMovements(LinkedHashMap<GoToPosition, Boolean> waypoints, int threshold, LinearOpMode teleOp){
        this.waypoints = waypoints;
        this.threshold = threshold;
        this.teleOP = teleOp;
    }

    public boolean runMovements() {
        GoToPosition currentWaypoint = null;

        for(GoToPosition waypoint : waypoints.keySet() ){
            currentWaypoint = waypoint;
            teleOP.telemetry.addData("Current Waypoint:",  currentWaypoint.targetPosition.x);
            break;
        }

        if(currentWaypoint == null){
            teleOP.telemetry.addLine("Sequential Movements finished");
            return true;
        }

        boolean finished = currentWaypoint.runWithPID(threshold);

        teleOP.telemetry.addData("Finished : ", finished);
        teleOP.telemetry.update();

        if(finished){
            waypoints.remove(currentWaypoint);
        }

        return false;
    }
}
