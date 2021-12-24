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
                new XyhVector(0, 20, Math.toRadians(0))
        );

        hardware.resetDriveEncoders();
        while (opModeIsActive() && !isStopRequested()) {
            hardware.odometry();
            gamepad.updateRobot();

            telemetry.addData("Position X", hardware.pos.x);
            telemetry.addData("Position Y", hardware.pos.y);
            telemetry.addData("Position H", hardware.pos.h);

            XyhVector outputCtrl = posVeloCtrl.runPID(hardware.pos);

//            hardware.frontLeftMotor.setPower(-0.5*(outputCtrl.x + outputCtrl.y + outputCtrl.h));
//            hardware.backLeftMotor.setPower(-0.5*(outputCtrl.x - outputCtrl.y + outputCtrl.h));
//            hardware.frontRightMotor.setPower(-0.5*(outputCtrl.x - outputCtrl.y - outputCtrl.h));
//            hardware.backRightMotor.setPower(-0.5*(outputCtrl.x + outputCtrl.y - outputCtrl.h));


            telemetry.addData("frontLeft", outputCtrl.x + outputCtrl.y + outputCtrl.h);
            telemetry.addData("backLeft", outputCtrl.x - outputCtrl.y + outputCtrl.h);
            telemetry.addData("frontRight", outputCtrl.x - outputCtrl.y - outputCtrl.h);
            telemetry.addData("backRight", outputCtrl.x + outputCtrl.y - outputCtrl.h);

            telemetry.addData("PID Output X", outputCtrl.x);
            telemetry.addData("PID Output Y", outputCtrl.y);
            telemetry.addData("PID Output H", outputCtrl.h);

            telemetry.addData("Servo Position", hardware.claw.getPosition());
            telemetry.update();
        }
    }
}
