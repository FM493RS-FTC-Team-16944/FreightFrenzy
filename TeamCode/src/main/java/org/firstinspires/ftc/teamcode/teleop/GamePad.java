package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.models.Mode;

public class GamePad {
    private Gamepad gamepad;
    private RobotHardware robot;

    public GamePad(RobotHardware robotHardware, Gamepad hardwareGamepad) {
        robot = robotHardware;
        gamepad = hardwareGamepad;
    }

    public void updateRobot() {
        if (gamepad.a) {
            switch (robot.currentMode) {
                case DRIVER_CONTROL:
                    robot.currentMode = Mode.AUTOMATIC_CONTROL;
                case AUTOMATIC_CONTROL:
                    robot.currentMode = Mode.DRIVER_CONTROL;
            }
        }

        if (gamepad.b) {

        }
        if (gamepad.x) {

        }
        if (gamepad.y) {


        }
        if (gamepad.left_trigger != 0) {

        } else if (gamepad.left_bumper) {
            switch (robot.intakeOperation) {
                case ON:
                    robot.intakeOperation = RobotHardware.Operation.OFF;
                case OFF:
                    robot.intakeOperation = RobotHardware.Operation.ON;

            }
        }
        if (gamepad.right_trigger != 0) {

        } else if (gamepad.right_bumper) {
            switch (robot.flyWheelOperation) {
                case ON:
                    robot.flyWheelOperation = RobotHardware.Operation.OFF;
                case OFF:
                    robot.flyWheelOperation = RobotHardware.Operation.ON;
            }
            if (gamepad.dpad_down){

             }
             if (gamepad.dpad_up){
             }
        }
    }
}