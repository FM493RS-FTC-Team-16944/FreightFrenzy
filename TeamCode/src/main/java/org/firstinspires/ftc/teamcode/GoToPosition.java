package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.models.XyhVector;

public class GoToPosition {
    private final Odometry odometry;
    private final TelemLog telemetry;

    public RobotMovement movement;
    public XyhVector targetPosition;
    public PositionVelocityCtrl forward;

    public GoToPosition(Robot robot, XyhVector targetPosition, TelemLog telemetry) {
        this.telemetry = telemetry;
        this.odometry = robot.hardware.odometry;
        this.movement = robot.movement;

        this.targetPosition = targetPosition;

        forward = new PositionVelocityCtrl(
                telemetry,
                targetPosition
        );
    }

    public boolean runWithPID(int threshold) {
        XyhVector position = odometry.getPosition();

        boolean completeX = Math.abs(position.x - targetPosition.x) <= threshold;
        boolean completeY = Math.abs(position.y - targetPosition.y) <= threshold;
        boolean completeH = Math.abs(position.h - targetPosition.h) <= Math.toRadians(threshold * 5);

        if(completeH && completeX && completeY) {
            this.telemetry.addLine("PID FINISHED");

            return false;
        } else {
            XyhVector forwardCtrl = forward.calculatePID(position);

            if(completeX) {
                forwardCtrl.x = 0;
            } else if(completeY) {
                forwardCtrl.y = 0;
            } else if(completeH) {
                forwardCtrl.h = 0;
            }

            odometry.setPosition(targetPosition);

            this.telemetry.addData("PID Output X", forwardCtrl.x);
            this.telemetry.addData("PID Output Y", forwardCtrl.y);
            this.telemetry.addData("PID Output H", forwardCtrl.h);

            this.movement.strafe(forwardCtrl.x, forwardCtrl.y, forwardCtrl.h);

            return true;
        }
    }
}
