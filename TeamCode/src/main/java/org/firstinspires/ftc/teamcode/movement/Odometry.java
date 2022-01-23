package org.firstinspires.ftc.teamcode.movement;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.TelemLog;
import org.firstinspires.ftc.teamcode.hardware.IMU;
import org.firstinspires.ftc.teamcode.hardware.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.models.Tuple;
import org.firstinspires.ftc.teamcode.models.XyhVector;

import java.util.concurrent.TransferQueue;
import java.util.logging.Level;

public class Odometry {
    private final double AUX_WIDTH = 11;
    private final double TRACK_WIDTH = 29;

    private final MecanumDriveTrain driveTrain;
    private final IMU imu;
    private final TelemLog telemetry;

    private double previousLeftPos;
    private double previousRightPos;
    private double previousAuxPos;

    private XyhVector pos = new XyhVector();

    public Odometry(RobotHardware robotHardware) {
        this.driveTrain = robotHardware.driveTrain;
        this.imu = robotHardware.imu;

        this.telemetry = robotHardware.telemetry;
    }

    public void update() {
        //double leftPosition = this.driveTrain.leftEncoder.getCurrentPosition(); //LEFT encoder is a fake encoder value
        double rightPosition = this.driveTrain.rightEncoder.getCurrentPosition();
        double horizontalPosition = this.driveTrain.auxEncoder.getCurrentPosition();

        Tuple<Double, Double> angles = this.imu.getAngle();
        double globalAngle = angles.x;
        double deltaAngle = angles.y;

        double leftPosition = (Math.toRadians(globalAngle)*TRACK_WIDTH)+rightPosition; // Generate left position via heading

        double deltaLeft = leftPosition - this.previousLeftPos;
        this.previousLeftPos = leftPosition; // Stores the current leftPosition to be used during the next update

        double deltaRight = rightPosition - this.previousRightPos;
        this.previousRightPos = rightPosition; // Stores the current rightPosition to be used during the next update h=a-b/trackwidth

        double deltaHorizontal = horizontalPosition - this.previousAuxPos;
        this.previousAuxPos = horizontalPosition; // Stores the current horizontalPosition to be used during the next update

        double horizontalOffset = AUX_WIDTH * deltaAngle;
        double relativeX = deltaHorizontal - horizontalOffset;
        double relativeY = (deltaLeft + deltaRight) / 2.0;

        pos.x += Math.cos(globalAngle) * relativeX - Math.sin(globalAngle) * relativeY;
        pos.y += Math.sin(globalAngle) * relativeX + Math.cos(globalAngle) * relativeY;
        pos.h = globalAngle;
    }

    public XyhVector getPosition() {
        return this.pos;
    }

    public void setPosition(XyhVector pos) {
        this.pos = pos;
    }

    public void updateOdometryReadings() {
        this.telemetry.addData("Odometry X Position Centimeters : " , this.pos.x);
        this.telemetry.addData("Odometry Y Position Centimeters : " , this.pos.y);
        this.telemetry.addData("Odometry H Position Centimeters : " , this.pos.h);
    }
}
