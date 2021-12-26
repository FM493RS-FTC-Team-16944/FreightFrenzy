package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.models.Lift;

public class LiftMacro implements Runnable {
    RobotMovement movement;
    Lift liftUpDown;

    LiftMacro(RobotMovement movement, Lift liftUpDown) {
        this.movement = movement;
        this.liftUpDown = liftUpDown;
    }

    @Override
    public void run() {
        if(liftUpDown == Lift.UP) {
            movement.toggleRaiseLift();

            try {
                Thread.sleep(4000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            movement.toggleRaiseLift();

            movement.toggleArm(0.5);
        } else {
            movement.toggleClaw(0.675);
            movement.toggleArm(0.98);

            movement.toggleLowerLift();

            try {
                Thread.sleep(5900);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            movement.toggleLowerLift();

            movement.toggleClaw(1);
        }
    }
}
