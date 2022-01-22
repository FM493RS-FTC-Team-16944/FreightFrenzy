package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.models.XyhVector;
import org.firstinspires.ftc.teamcode.teleop.TeleOP;

public class GoToPosition {
    public TeleOP teleOP;
    public RobotHardware hardware;
    public RobotMovement movement;
    public RobotHardware correctedHardware;
    public XyhVector targetPosition;
    public PositionVelocityCtrl forward;

    public GoToPosition(Robot robot, XyhVector targetPosition, TeleOP teleOP) {
        this.teleOP = teleOP;
        this.hardware = robot.hardware;
        this.movement = robot.movement;
        this.correctedHardware = robot.hardware;

        this.targetPosition = targetPosition;

        forward = new PositionVelocityCtrl(
                teleOP,
                targetPosition
        );
    }

    public boolean runWithPID(int threshold) {
        boolean complete_x = Math.abs(correctedHardware.pos.x - targetPosition.x) <= threshold;
        boolean complete_y = Math.abs(correctedHardware.pos.y - targetPosition.y) <= threshold;
        boolean complete_h = Math.abs(correctedHardware.pos.h - targetPosition.h) <= Math.toRadians(threshold * 5);

        if(complete_h && complete_x && complete_y) {
            teleOP.telemetry.addLine("PID FINISHED");

            return false;
        } else {
            XyhVector forwardCtrl = forward.calculatePID(correctedHardware.pos);

            if(complete_x) {
                correctedHardware.pos.x = targetPosition.x;
                forwardCtrl.x = 0;
            } else if(complete_y) {
                correctedHardware.pos.y = targetPosition.y;
                forwardCtrl.y = 0;
            } else if(complete_h) {
                correctedHardware.pos.h = targetPosition.h;
                forwardCtrl.h = 0;
            }

            teleOP.telemetry.addData("PID Output X", forwardCtrl.x);
            teleOP.telemetry.addData("PID Output Y", forwardCtrl.y);
            teleOP.telemetry.addData("PID Output H", forwardCtrl.h);

            movement.strafe(forwardCtrl.x, forwardCtrl.y, forwardCtrl.h);

            return true;
        }
    }




}
