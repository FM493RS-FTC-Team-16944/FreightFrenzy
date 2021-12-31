package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.sun.source.tree.Tree;

import org.firstinspires.ftc.teamcode.GamePad;
import org.firstinspires.ftc.teamcode.GoToPosition;
import org.firstinspires.ftc.teamcode.PositionVelocityCtrl;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.SequentialMovements;
import org.firstinspires.ftc.teamcode.XyhVector;
import org.firstinspires.ftc.teamcode.GoToPosition;

import java.util.TreeMap;

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
        XyhVector targetVector = new XyhVector(20,0,Math.toRadians(0));
        GoToPosition runToTarget = new GoToPosition(robot, targetVector, this);
        XyhVector secondTarget = new XyhVector(0,0,Math.toRadians(0));
        GoToPosition runBackToOrigin = new GoToPosition(robot, secondTarget, this);
        TreeMap<GoToPosition, Boolean> waypoints = new TreeMap<GoToPosition, Boolean>();
        waypoints.put(runToTarget, false);
        waypoints.put(runBackToOrigin, false);
        SequentialMovements path = new SequentialMovements(waypoints, 3);
        int navigator = 0;


        hardware.resetDriveEncoders();
        while (opModeIsActive() && !isStopRequested()) {
            hardware.odometry();
            gamepad.updateRobot();

            path.runMovements();

            telemetry.addData("Position X", hardware.pos.x);
            telemetry.addData("Position Y", hardware.pos.y);
            telemetry.addData("Posiion H", Math.toDegrees(hardware.pos.h));


            telemetry.addData("Claw Position", hardware.claw.getPosition());
            telemetry.addData("Arm Position", hardware.arm.getPosition());
            telemetry.update();
        }
    }
}