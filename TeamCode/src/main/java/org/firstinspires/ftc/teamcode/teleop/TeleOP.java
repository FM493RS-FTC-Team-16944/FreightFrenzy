package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotHardware;


@TeleOp(name = "GamePad")
public class TeleOP extends LinearOpMode {

    Mode currentMode = Mode.DRIVER_CONTROL;
    DcMotor frontRightMotor, frontLeftMotor, backRightMotor, backLeftMotor = null;
    DcMotor intake, lift, flyWheel;
    Servo carriage;
    private RobotHardware robot;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new RobotHardware(hardwareMap);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            robot.odometry();
            //TODO: Display odom telemetry readings (pos.x,pos.y,pos.h)
            //TODO: Deleted mode switch, put in again
        }
    }

    enum Operation {
        ON,
        OFF
    }
    enum Mode {
        DRIVER_CONTROL,
        AUTOMATIC_CONTROL
    }

    class IntakeControl {
        Operation operation = Operation.OFF;
    }

    class LiftPositions {
        final double downPosition = 0.0;
        final double upPosition = 1.0;
    }

    class FlyWheelControl {
        Operation operation = Operation.OFF;
    }
}
