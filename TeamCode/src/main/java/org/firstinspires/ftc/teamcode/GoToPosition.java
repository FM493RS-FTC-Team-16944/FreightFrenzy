package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.teleop.TeleOP;

public class GoToPosition {
    public TeleOP teleOP;
    public RobotHardware hardware;
    public RobotMovement movement;
    public XyhVector targetPosition;
    public PositionVelocityCtrl forward
;
    public GoToPosition(Robot robot, XyhVector targetPosition, TeleOP teleOP){
        this.teleOP = teleOP;
        this.hardware = robot.hardware;
        this.movement = robot.movement;

        this.targetPosition = targetPosition;

        forward = new PositionVelocityCtrl(
                teleOP,
                targetPosition
        );
    }


    public void runWithPID(int threshold){
        if(Math.abs(hardware.pos.x - targetPosition.x) >= threshold || Math.abs(hardware.pos.y - targetPosition.y) >= threshold ||  Math.abs(hardware.pos.h - targetPosition.h) >= Math.toRadians(threshold)) {
            XyhVector forwardCtrl= forward.calculatePID(hardware.pos);
            movement.strafe(forwardCtrl.x, forwardCtrl.y, forwardCtrl.h);
            teleOP.telemetry.addData("PID Output X", forwardCtrl.x);
            teleOP.telemetry.addData("PID Output Y", forwardCtrl.y);
            teleOP.telemetry.addData("PID Output H", forwardCtrl.h);
        }else{
            teleOP.telemetry.addLine("PID FINISHED");
        }
    }




}
