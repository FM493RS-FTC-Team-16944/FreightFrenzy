package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.GamePad;
import org.firstinspires.ftc.teamcode.GoToPosition;
import org.firstinspires.ftc.teamcode.PositionVelocityCtrl;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.XyhVector;
import org.firstinspires.ftc.teamcode.GoToPosition;

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
        XyhVector targetVector = new XyhVector(0,20,Math.toRadians(0));
        GoToPosition runToTarget = new GoToPosition(robot, targetVector, this);

        hardware.resetDriveEncoders();
        while (opModeIsActive() && !isStopRequested()) {
            hardware.odometry();
            gamepad.updateRobot();

            runToTarget.runWithPID(3);


            telemetry.addData("Position X", hardware.pos.x);
            telemetry.addData("Position Y", hardware.pos.y);
            telemetry.addData("Posiion H", Math.toDegrees(hardware.pos.h));


            telemetry.addData("Claw Position", hardware.claw.getPosition());
            telemetry.addData("Arm Position", hardware.arm.getPosition());
            telemetry.update();
        }
    }
}