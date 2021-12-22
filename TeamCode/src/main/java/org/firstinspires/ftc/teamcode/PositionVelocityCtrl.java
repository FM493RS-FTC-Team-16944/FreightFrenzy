package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teleop.TeleOP;

public class PositionVelocityCtrl {
    public TeleOP teleOP;

    private final ElapsedTime PIDTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private final XyhVector targetPosition;

    public final double propGain = 0.5f;
    public final double intGain = 0.002f;
    public final double derivGain = 0.005f;

    public static XyhVector integralVelo = new XyhVector();
    public static XyhVector errPos = new XyhVector();

    public PositionVelocityCtrl(TeleOP teleOP, XyhVector targetPosition) {
        this.teleOP = teleOP;

        this.targetPosition = targetPosition;
    }

    public XyhVector runPID(XyhVector currentPosition) {
        double currentPosErrX = targetPosition.x - currentPosition.x;
        double currentPosErrY = targetPosition.y - currentPosition.y;
        double currentPosErrH = targetPosition.h - currentPosition.h;

        integralVelo.x += errPos.x * PIDTimer.time();
        integralVelo.y += errPos.y * PIDTimer.time();
        integralVelo.h += errPos.h * PIDTimer.time();

        XyhVector posDerivative = new XyhVector(
                (currentPosErrX - errPos.x) / PIDTimer.time(),
                (currentPosErrY - errPos.y) / PIDTimer.time(),
                (currentPosErrH - errPos.h) / PIDTimer.time()
        );

        double outX = propGain * currentPosErrX +
                intGain * integralVelo.x +
                derivGain * posDerivative.x;

        double outY = propGain * currentPosErrY +
                intGain * integralVelo.y +
                derivGain * posDerivative.y;

        double outH = propGain * currentPosErrH +
                intGain * integralVelo.h +
                derivGain * posDerivative.h;

        double x_rotated = outX * Math.cos(currentPosition.h) - outY * Math.sin(currentPosition.h);
        double y_rotated = outX * Math.sin(currentPosition.h) + outY * Math.cos(currentPosition.h);


        errPos.x = currentPosErrX;
        errPos.y = currentPosErrY;
        errPos.h = currentPosErrH;
        PIDTimer.reset();

        return new XyhVector(x_rotated, y_rotated, outH);
    }
}
