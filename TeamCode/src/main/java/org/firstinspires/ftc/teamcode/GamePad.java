package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.models.Mode;

public class GamePad {
    private final Gamepad gamepad;
    private final RobotHardware hardware;
    private final DriveMovement movement;

    private boolean previousX = false;

    public GamePad(Robot robot, Gamepad hardwareGamepad) {
        this.hardware = robot.hardware;
        this.movement = robot.movement;

        this.gamepad = hardwareGamepad;
    }

    public void updateRobot() {
        if(hardware.currentMode == Mode.DRIVER_CONTROL) {
            double y = -gamepad.left_stick_y; // Remember, this is reversed!
            double x = gamepad.left_stick_x;
            double rx = gamepad.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y - x + rx) / denominator;
            double backLeftPower = (y + x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            hardware.frontLeftMotor.setPower(frontLeftPower);
            hardware.backLeftMotor.setPower(backLeftPower);
            hardware.frontRightMotor.setPower(frontRightPower);
            hardware.backRightMotor.setPower(backRightPower);
        }

        // change mode
        if (gamepad.a) {
            switch (hardware.currentMode) {
                case DRIVER_CONTROL:
                    hardware.currentMode = Mode.AUTOMATIC_CONTROL;

                case AUTOMATIC_CONTROL:
                    hardware.currentMode = Mode.DRIVER_CONTROL;
            }
        }

        // no clue
        if (gamepad.b) {

        }

        if (gamepad.x && gamepad.x != previousX) {
            movement.toggleClaw();
        }

        previousX = gamepad.x;
        
        if (gamepad.y) {
        }

        // intake
        if (gamepad.left_trigger != 0) {
            movement.activateIntake(gamepad.left_trigger);
        } else if (gamepad.left_bumper) {
            movement.activateIntake();
        }

        // flywheel
        if (gamepad.right_trigger != 0) {
            movement.activateFlywheel(gamepad.right_trigger);
        } else if (gamepad.right_bumper) {
            movement.activateFlywheel();
        }

        if (gamepad.dpad_up || gamepad.dpad_down) {
            movement.toggleRaiseLift();
        }

        if(gamepad.dpad_left || gamepad.dpad_right) {
            movement.toggleTranslateLift();
        } else {
            movement.toggleTranslateLift(0);
        }
    }
}