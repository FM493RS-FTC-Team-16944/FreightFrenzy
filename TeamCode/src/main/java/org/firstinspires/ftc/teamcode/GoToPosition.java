package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.models.XyhVector;

public class GoToPosition {
    public LinearOpMode opMode;
    public RobotHardware hardware;
    public RobotMovement movement;
    public RobotHardware correctedHardware;
    public XyhVector targetPosition;
    public PositionVelocityCtrl forward;
    public boolean complete;
    public boolean turn;
    private int i;

    public GoToPosition(Robot robot, XyhVector targetPosition, LinearOpMode opMode, boolean turn) {
        this.opMode = opMode;
        this.hardware = robot.hardware;
        this.movement = robot.movement;
        this.correctedHardware = robot.hardware;
        this.complete = false;
        this.turn = turn;
        i = 0;


        this.targetPosition = targetPosition;

        forward = new PositionVelocityCtrl(
                opMode,
                targetPosition
        );
    }

    public boolean runWithPID(int threshold) {
        boolean complete_x = Math.abs(correctedHardware.pos.x - targetPosition.x) <= threshold;
        boolean complete_y = Math.abs(correctedHardware.pos.y - targetPosition.y) <= threshold;
        boolean complete_h = Math.abs(correctedHardware.pos.h - targetPosition.h) <= Math.toRadians(5);

        //opMode.telemetry.addData("Current X : ", correctedHardware.pos.x);
        //opMode.telemetry.addData("Delta X : ", (correctedHardware.pos.x - targetPosition.x));
        //opMode.telemetry.addData("Delta Y : ", Math.abs(correctedHardware.pos.y - targetPosition.y));
        //opMode.telemetry.addData("Delta H : ", Math.abs(correctedHardware.pos.h - targetPosition.h));
        //opMode.telemetry.addData("Threshold : ", threshold);
        //opMode.telemetry.addData("Threshold H : ", Math.toRadians(5));


        XyhVector forwardCtrl = forward.calculatePID(correctedHardware.pos);

        if(complete_x) {
            //correctedHardware.pos.x = targetPosition.x;
            forwardCtrl.x = 0;
        } else if(complete_y) {
            //correctedHardware.pos.y = targetPosition.y;
            forwardCtrl.y = 0;
            } else if(complete_h) {
            correctedHardware.pos.h = targetPosition.h;
            forwardCtrl.h = 0;
        }

        // teleOP.telemetry.addData("PID Output X", forwardCtrl.x);
        // teleOP.telemetry.addData("PID Output Y", forwardCtrl.y);
        //opMode.telemetry.addData("PID Output H", forwardCtrl.h);

        if (turn) {
            forwardCtrl.x = 0;
            forwardCtrl.y = 0;
            //correctedHardware.pos.x = targetPosition.x;
            //correctedHardware.pos.y = targetPosition.y;
            if (complete_h) {
                this.complete = true;
                if (i == 0) {
                    correctedHardware.pos = targetPosition;
                    i++;
                }
            } else {
                opMode.telemetry.addLine("Not finished turning yet man");
            }
        }

        if (complete_x && complete_y && complete_h) {
            this.complete = true;
            if (i==0) {
                correctedHardware.pos = targetPosition;
                i++;
            }
        }

        if (!this.complete) {
            movement.strafe(forwardCtrl.x, -forwardCtrl.y, forwardCtrl.h);
        } else {
            movement.strafe(0,0,0);
        }

        return this.complete;
    }
}


