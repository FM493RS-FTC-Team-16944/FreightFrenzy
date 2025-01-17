package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.models.Lift;
import org.firstinspires.ftc.teamcode.models.Mode;

public class GamePad {
    private final Gamepad gamepad;

    private final RobotHardware hardware;
    private final RobotMovement movement;

    private boolean previousX = false;
    private boolean previousY = false;
    private boolean previousIn = false;
    private boolean previousFly = false;
    private boolean previousUp = false;
    private boolean previousDown = false;
    private int i;

    public GamePad(Robot robot, Gamepad hardwareGamepad) {
        this.hardware = robot.hardware;
        this.movement = robot.movement;

        this.gamepad = hardwareGamepad;
        this.i = 0;
    }

    public void updateRobot() {
        if(hardware.currentMode == Mode.DRIVER_CONTROL) {
            double x = gamepad.left_stick_x;
            double y = gamepad.left_stick_y; // Remember, this is reversed!
            double h = gamepad.right_stick_x;

            movement.strafeR(x, y, h);
        }

        // change mode
        if (gamepad.a) {
            this.hardware.resetAngle();
            /*
            switch (hardware.currentMode) {
                case DRIVER_CONTROL:
                    hardware.currentMode = Mode.AUTOMATIC_CONTROL;

                case AUTOMATIC_CONTROL:
                    hardware.currentMode = Mode.DRIVER_CONTROL;
            }

             */
        }

        // no clue
        if (gamepad.b) {

        }

        // claw
        if (gamepad.x && gamepad.x != previousX) {
            movement.toggleClaw();
        }

        previousX = gamepad.x;

        // arm
        if (gamepad.y && gamepad.y != previousY) {
            movement.toggleArm();
        }

        previousY = gamepad.y;

        // intake
        if (gamepad.left_trigger != 0) {
            movement.activateIntake(gamepad.left_trigger);
        } else if (gamepad.left_bumper && gamepad.left_bumper != previousIn) {
            movement.activateIntake();
        }

        previousIn = gamepad.left_bumper;

        // flywheel
        if (gamepad.right_trigger != 0) {
            movement.activateFlywheel(gamepad.right_trigger);
        } else if (gamepad.right_bumper && gamepad.right_bumper != previousFly) {
            movement.activateFlywheel();
        }

        previousFly = gamepad.right_bumper;

        LiftMacro liftMacroUp = new LiftMacro(movement, Lift.UP, 2500);
        Thread t1 = new Thread(liftMacroUp);
        LiftMacro liftMacroDown = new LiftMacro(movement, Lift.DOWN, 2500);
        Thread t2 = new Thread(liftMacroDown);


        if (gamepad.dpad_up && gamepad.dpad_up != previousUp) {
            t1.start();
        } else if (gamepad.dpad_down && gamepad.dpad_down != previousDown) {
            t2.start();
        } else if (gamepad.dpad_right) {
            movement.moveLift(0.5);
            i = 0;
        } else if (gamepad.dpad_left) {
            movement.moveLift(-0.2);
            i = 0;
        } else if (gamepad.b){
            movement.moveLift(0);
        }

        previousUp = gamepad.dpad_up;
        previousDown = gamepad.dpad_down;


    }
}