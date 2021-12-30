package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teleop.TeleOP;

public class DifferentialDriveDistancePID {
    public TeleOP teleOP;
    private final ElapsedTime PIDTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    private int targetPosition = 0;

    public final double propGain = 0.05f;
    public final double intGain = 0.01f;
    public final double derivGain = 0.01f;

    public static double integralSum = 0;
    public static XyhVector errPos = new XyhVector(0,0,0);

    public DifferentialDriveDistancePID(TeleOP teleOP, int threshold) {
        this.teleOP = teleOP;
        this.targetPosition = threshold;
    }

//    public double calculatePID(double currentDistance) {
//        double currentPosErrX = targetPosition - currentDistance;
//
//        teleOP.telemetry.addData("Target X", targetPosition.x);
//        teleOP.telemetry.addData("Target Y", targetPosition.y);
//        teleOP.telemetry.addData("Target H", targetPosition.h);
//
//        teleOP.telemetry.addData("Error X", currentPosErrX);
//        teleOP.telemetry.addData("Error Y", currentPosErrY);
//        teleOP.telemetry.addData("Error H", currentPosErrH);
//
//
//        integralSum.x += currentPosErrX * PIDTimer.time();
//        integralSum.y += currentPosErrY * PIDTimer.time();
//        integralSum.h += currentPosErrH * PIDTimer.time();
//
//
//        XyhVector posDerivative = new XyhVector(
//                (currentPosErrX - errPos.x) / PIDTimer.time(),
//                (currentPosErrY - errPos.y) / PIDTimer.time(),
//                (currentPosErrH - errPos.h) / PIDTimer.time()
//        );
//
//        double outX = propGain * currentPosErrX +
//                intGain * integralSum.x +
//                derivGain * posDerivative.x;
//
//        double outY = propGain * currentPosErrY +
//                intGain * integralSum.y +
//                derivGain * posDerivative.y;
//
//        double outH = propGain * currentPosErrH +
//                intGain * integralSum.h +
//                derivGain * posDerivative.h;
//
//        double x_rotated = outX * Math.cos(currentPosition.h) - outY * Math.sin(currentPosition.h);
//        double y_rotated = outX * Math.sin(currentPosition.h) + outY * Math.cos(currentPosition.h);
//
//        errPos.x = currentPosErrX;
//        errPos.y = currentPosErrY;
//        errPos.h = currentPosErrH;
//        PIDTimer.reset();
//        //return new XyhVector(currentPosErrX, currentPosErrY, currentPosErrH);
//        return new XyhVector(x_rotated, y_rotated, outH);
//    }
}
