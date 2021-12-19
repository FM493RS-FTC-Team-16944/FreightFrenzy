package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.GamePad;
import org.firstinspires.ftc.teamcode.PositionVelocityCtrl;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.XyhVector;

@TeleOp(name = "TeleOp")
public class TeleOP extends LinearOpMode {
    private Robot robot;

    public RobotHardware hardware;
    public GamePad gamepad;

    @Override
    public void runOpMode() {
        waitForStart();

        robot = new Robot(this);
        hardware = robot.hardware;
        gamepad = new GamePad(robot, gamepad1);

        PositionVelocityCtrl posVeloCtrl = new PositionVelocityCtrl(
                this,
                new XyhVector(60, 0, Math.toRadians(90))
        );

        while (opModeIsActive() && !isStopRequested()) {
            hardware.odometry();
            gamepad.updateRobot();

            telemetry.addData("Position X", hardware.pos.x);
            telemetry.addData("Position Y", hardware.pos.y);
            telemetry.addData("Position H", hardware.pos.h);

            XyhVector xyhVector = posVeloCtrl.runPID(hardware.pos);

            telemetry.addData("PID Output X", xyhVector.x);
            telemetry.addData("PID Output Y", xyhVector.y);
            telemetry.addData("PID Output H", xyhVector.h);
            telemetry.update();
        }
    }
}
