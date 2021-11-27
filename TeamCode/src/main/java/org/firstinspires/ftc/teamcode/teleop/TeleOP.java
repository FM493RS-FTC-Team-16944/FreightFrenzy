package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotHardware;


@TeleOp(name = "GamePad")
public class TeleOP extends LinearOpMode {
    private RobotHardware robot;
    private GamePad gamepad;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotHardware(hardwareMap);
        gamepad = new GamePad(robot, gamepad1);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            robot.odometry();
            gamepad.updateRobot();

            telemetry.addData("Position X", robot.pos.x);
            telemetry.addData("Position Y", robot.pos.y);
            telemetry.addData("Position H", robot.pos.h);
        }
    }
}
