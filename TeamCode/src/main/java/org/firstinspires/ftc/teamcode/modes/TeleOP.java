package org.firstinspires.ftc.teamcode.modes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.GamePad;
import org.firstinspires.ftc.teamcode.GoToPosition;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.TelemLog;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.SequentialMovements;
import org.firstinspires.ftc.teamcode.models.XyhVector;

import java.util.LinkedHashMap;

@TeleOp(name = "TeleOp")
public class TeleOP extends LinearOpMode {
    public Robot robot;

    public RobotHardware hardware;
    public GamePad gamepad;
    public TelemLog telemLog;

    @Override
    public void runOpMode() {
        waitForStart();

        this.robot = new Robot(this);
        this.hardware = robot.hardware;
        this.gamepad = new GamePad(robot, gamepad1);
        this.telemLog = robot.telemLog;

        XyhVector targetVector = new XyhVector(50,0,Math.toRadians(0));
        GoToPosition runToTarget = new GoToPosition(this.robot, targetVector, this.telemLog);

        XyhVector secondTarget = new XyhVector(0,0,Math.toRadians(0));
        GoToPosition runBackToOrigin = new GoToPosition(this.robot, secondTarget, this.telemLog);

        LinkedHashMap<GoToPosition, Boolean> waypoints = new LinkedHashMap<>();
        waypoints.put(runToTarget, false);
        waypoints.put(runBackToOrigin, false);

        SequentialMovements path = new SequentialMovements(waypoints, 3, this.telemLog);

        while (opModeIsActive() && !isStopRequested()) {
            hardware.odometry.update();
            gamepad.updateRobot();

            path.runMovements();

            hardware.outputReadings();

            telemetry.update();
        }
    }
}