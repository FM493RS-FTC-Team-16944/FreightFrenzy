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
        if(gamepad.a) {
            switch (robot.currentMode) {
                case DRIVER_CONTROL:
                    robot.currentMode = Mode.AUTOMATIC_CONTROL;
                case AUTOMATIC_CONTROL:
                    robot.currentMode = Mode.DRIVER_CONTROL;
            }
        }
    }
}
