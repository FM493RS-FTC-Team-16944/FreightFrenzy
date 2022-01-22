package org.firstinspires.ftc.teamcode.modes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.GamePad;
import org.firstinspires.ftc.teamcode.GoToPosition;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.SequentialMovements;
import org.firstinspires.ftc.teamcode.models.XyhVector;

import java.util.LinkedHashMap;

@TeleOp(name = "TeleOp")
public class TeleOP extends LinearOpMode {
    public Robot robot;

    public RobotHardware hardware;
    public GamePad gamepad;

    @Override
    public void runOpMode() {
        waitForStart();

        robot = new Robot(this);
        hardware = robot.hardware;
        gamepad = new GamePad(robot, gamepad1);

        XyhVector targetVector = new XyhVector(50,0,Math.toRadians(0));
        GoToPosition runToTarget = new GoToPosition(robot, targetVector, this.telemetry);

        XyhVector secondTarget = new XyhVector(0,0,Math.toRadians(0));
        GoToPosition runBackToOrigin = new GoToPosition(robot, secondTarget, this.telemetry);

        LinkedHashMap<GoToPosition, Boolean> waypoints = new LinkedHashMap<>();
        waypoints.put(runToTarget, false);
        waypoints.put(runBackToOrigin, false);

        SequentialMovements path = new SequentialMovements(waypoints, 3, this.telemetry);

        while (opModeIsActive() && !isStopRequested()) {
            hardware.odometry.update();
            gamepad.updateRobot();

            path.runMovements();

            hardware.odometry.outputOdometryReadings();
            hardware.manipulator.outputPositionReadings();
            hardware.driveTrain.outputEncoderReadings();
            hardware.imu.outputIMUReadings();

            telemetry.update();
        }
    }
}