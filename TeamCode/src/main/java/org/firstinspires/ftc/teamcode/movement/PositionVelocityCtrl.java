package org.firstinspires.ftc.teamcode.movement;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.TelemLog;
import org.firstinspires.ftc.teamcode.models.XyhVector;

public class PositionVelocityCtrl {
    private final double PROP_GAIN = 0.05f;
    private final double INT_GAIN = 0.0001f;
    private final double DERIV_GAIN = 0.01f;

    private final ElapsedTime PIDTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    private final XyhVector targetPosition;
    private final TelemLog telemetry;

    public XyhVector integralSum = new XyhVector(); // TODO: THIS MIGHT BE A BUG FOR IT BEING STATIC
    public XyhVector errPos = new XyhVector(0,0,0); // TODO: THIS TOO

    public PositionVelocityCtrl(TelemLog telemetry, XyhVector targetPosition) {
        this.telemetry = telemetry;

        this.targetPosition = targetPosition;
    }

    public XyhVector calculatePID(XyhVector currentPosition) {
        XyhVector currentPosErr = targetPosition.subtract(currentPosition);

        this.telemetry.addData("PID Target X", targetPosition.x);
        this.telemetry.addData("PID Target Y", targetPosition.y);
        this.telemetry.addData("PID Target H", targetPosition.h);

        this.telemetry.addData("PID Error X", currentPosErr.x);
        this.telemetry.addData("PID Error Y", currentPosErr.y);
        this.telemetry.addData("PID Error H", currentPosErr.h);

        //this.telemetry.update();

        integralSum.x += currentPosErr.x * PIDTimer.time();
        integralSum.y += currentPosErr.y * PIDTimer.time();
        integralSum.h += currentPosErr.h * PIDTimer.time();

        XyhVector deltaErr = currentPosErr.subtract(errPos);

        XyhVector posDerivative = new XyhVector(
                (deltaErr.x) / PIDTimer.time(),
                (deltaErr.y) / PIDTimer.time(),
                (deltaErr.h) / PIDTimer.time()
        );

        double outX = -PROP_GAIN * currentPosErr.x +
                INT_GAIN * integralSum.x +
                DERIV_GAIN * posDerivative.x;

        double outY = -PROP_GAIN * currentPosErr.y +
                INT_GAIN * integralSum.y +
                DERIV_GAIN * posDerivative.y;

        double outH = -PROP_GAIN * currentPosErr.h +
                INT_GAIN * integralSum.h +
                DERIV_GAIN * posDerivative.h;

        double x_rotated = outX * Math.cos(currentPosition.h) - outY * Math.sin(currentPosition.h);
        double y_rotated = outX * Math.sin(currentPosition.h) + outY * Math.cos(currentPosition.h);

        errPos.x = currentPosErr.x;
        errPos.y = currentPosErr.y;
        errPos.h = currentPosErr.h;

        PIDTimer.reset();

        return new XyhVector(x_rotated, y_rotated, outH);
    }
}