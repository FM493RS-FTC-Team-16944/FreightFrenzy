package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.LinkedHashMap;

public class SequentialMovements {
    private final LinkedHashMap<GoToPosition, Boolean> waypoints;
    private final int threshold;
    private final Telemetry telemetry;

    public SequentialMovements(
            LinkedHashMap<GoToPosition, Boolean> waypoints,
            int threshold,
            Telemetry telemetry
    ) {
        this.waypoints = waypoints;
        this.threshold = threshold;
        this.telemetry = telemetry;
    }

    public void runMovements() {
        GoToPosition currentWaypoint = null;

        for(GoToPosition waypoint : this.waypoints.keySet()) {
            if(!this.waypoints.get(waypoint)) {
                currentWaypoint = waypoint;

                this.telemetry.addData("Current Waypoint :",  currentWaypoint.targetPosition.x);
                break;
            }
        }

        if(currentWaypoint == null) {
            this.telemetry.addLine("Sequential Movements finished");

            return;
        }

        boolean finished = currentWaypoint.runWithPID(threshold);

        if(!finished) {
            waypoints.remove(currentWaypoint);
        }
    }
}
