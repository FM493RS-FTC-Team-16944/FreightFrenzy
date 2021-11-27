package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotHardware;


@TeleOp(name = "GamePad")
public class TeleOP extends LinearOpMode {

    private RobotHardware robot;

    enum Operation {
        ON,
        OFF
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

    Mode currentMode = Mode.DRIVER_CONTROL;

    DcMotor frontRightMotor, frontLeftMotor, backRightMotor, backLeftMotor = null;
    DcMotor intake, lift, flyWheel;
    Servo carriage;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new RobotHardware(hardwareMap);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            robot.odometry();
        }
    }


    enum Mode {
        DRIVER_CONTROL,
        AUTOMATIC_CONTROL
    }
}
