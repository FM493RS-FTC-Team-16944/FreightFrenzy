package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.models.XyhVector;

public class PositionVelocityCtrl {
    public LinearOpMode opMode;

    private final ElapsedTime PIDTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    public final XyhVector targetPosition;

    public final double propGain = 0.1f;
    public final double intGain = 0.0005f;
    public final double derivGain = 0.01f;

    public static XyhVector integralSum = new XyhVector();
    public static XyhVector errPos = new XyhVector(0,0,0);

    public PositionVelocityCtrl(LinearOpMode opMode, XyhVector targetPosition) {
        this.opMode = opMode;

        this.targetPosition = targetPosition;
    }

    public double angleWrap(double radians) {
        if (radians > Math.PI) {
            radians -= 2 * Math.PI;
        }

        if (radians < -Math.PI) {
            radians += 2 * Math.PI;
        }

        return radians;
    }

    public XyhVector calculatePID(XyhVector currentPosition) {
        double currentPosErrX = targetPosition.x - currentPosition.x;
        double currentPosErrY = targetPosition.y - currentPosition.y;
        double currentPosErrH = targetPosition.h - currentPosition.h;

        // teleOP.telemetry.addData("Target X", targetPosition.x);
        // teleOP.telemetry.addData("Target Y", targetPosition.y);
        // teleOP.telemetry.addData("Target H", targetPosition.h);

        // teleOP.telemetry.addData("Error X", currentPosErrX);
        // teleOP.telemetry.addData("Error Y", currentPosErrY);
        // teleOP.telemetry.addData("Error H", currentPosErrH);


        integralSum.x += currentPosErrX * PIDTimer.time();
        integralSum.y += currentPosErrY * PIDTimer.time();
        integralSum.h += currentPosErrH * PIDTimer.time();


        XyhVector posDerivative = new XyhVector(
                (currentPosErrX - errPos.x) / PIDTimer.time(),
                (currentPosErrY - errPos.y) / PIDTimer.time(),
                (currentPosErrH - errPos.h) / PIDTimer.time()
        );

        double outX = propGain * currentPosErrX +
                intGain * integralSum.x +
                derivGain * posDerivative.x;

        double outY = propGain * currentPosErrY +
                intGain * integralSum.y +
                derivGain * posDerivative.y;

        double outH = (propGain * currentPosErrH +
                intGain * integralSum.h +
                derivGain * posDerivative.h) * 30;

        double x_rotated = outX * Math.cos(currentPosition.h) - outY * Math.sin(currentPosition.h);
        double y_rotated = outX * Math.sin(currentPosition.h) + outY * Math.cos(currentPosition.h);

        errPos.x = currentPosErrX;
        errPos.y = currentPosErrY;
        errPos.h = currentPosErrH;

        PIDTimer.reset();

        return new XyhVector(x_rotated, y_rotated, outH);
    }
}
