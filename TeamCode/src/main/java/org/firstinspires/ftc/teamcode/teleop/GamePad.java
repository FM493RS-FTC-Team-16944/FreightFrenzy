package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "GamePad")
public class GamePad extends LinearOpMode {
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
        frontRightMotor = hardwareMap.dcMotor.get("FrontRight");
        frontLeftMotor = hardwareMap.dcMotor.get("FrontLeft");

        backRightMotor = hardwareMap.dcMotor.get("BackRight");
        backLeftMotor = hardwareMap.dcMotor.get("BackLeft");

        intake = hardwareMap.dcMotor.get("Intake");
        lift = hardwareMap.dcMotor.get("Lift");
        flyWheel = hardwareMap.dcMotor.get("Flywheel");

        waitForStart();

        if (isStopRequested()) return;

        gamepad1
        while (opModeIsActive() && !isStopRequested()) {
            // We follow different logic based on whether we are in manual driver control or switch
            // control to the automatic mode
            switch (currentMode) {

            }
        }
    }


    enum Mode {
        DRIVER_CONTROL,
        AUTOMATIC_CONTROL
    }
}
