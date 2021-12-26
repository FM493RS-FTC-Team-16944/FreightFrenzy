package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.GamePad;
import org.firstinspires.ftc.teamcode.PositionVelocityCtrl;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.XyhVector;

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
        PositionVelocityCtrl posVeloCtrl = new PositionVelocityCtrl(
                this,
                new XyhVector(0, 0, Math.toRadians(90))
        );

        hardware.resetDriveEncoders();
        while (opModeIsActive() && !isStopRequested()) {
            hardware.odometry();
            gamepad.updateRobot();

            telemetry.addData("Position X", hardware.pos.x);
            telemetry.addData("Position Y", hardware.pos.y);
            telemetry.addData("Position H", Math.toDegrees(hardware.pos.h));

            XyhVector outputCtrl = posVeloCtrl.runPID(hardware.pos);


//            double frontLeftPower = (x  + y + h) / denominator;
//            double backLeftPower = (x - y - h) / denominator;
//            double frontRightPower = (x + y - h) / denominator;
//            double backRightPower = (x - y + h) / denominator;

//           sa

            telemetry.addData("frontLeft", outputCtrl.x + outputCtrl.y + outputCtrl.h);
            telemetry.addData("backLeft", outputCtrl.x - outputCtrl.y + outputCtrl.h);
            telemetry.addData("frontRight", outputCtrl.x - outputCtrl.y - outputCtrl.h);
            telemetry.addData("backRight", outputCtrl.x + outputCtrl.y - outputCtrl.h);

            telemetry.addData("PID Output X", outputCtrl.x);
            telemetry.addData("PID Output Y", outputCtrl.y);
            telemetry.addData("PID Output H", outputCtrl.h);

            telemetry.addData("Claw Position", hardware.claw.getPosition());
            telemetry.addData("Arm Position", hardware.arm.getPosition());
            telemetry.update();
        }
    }
}