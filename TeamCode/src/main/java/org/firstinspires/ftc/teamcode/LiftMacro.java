package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.models.Lift;

public class LiftMacro implements Runnable {
    RobotMovement movement;
    Lift liftUpDown;
    public boolean complete;

    public LiftMacro(RobotMovement movement, Lift liftUpDown) {
        this.movement = movement;
        this.liftUpDown = liftUpDown;
    }       

    @Override
    public void run() {
        if(liftUpDown == Lift.UP) {
            movement.moveLift(0.5);

            try {
                Thread.sleep(2500);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            movement.moveLift(0);

            movement.toggleArm(0.5);
            this.complete = true;
        } else {
            try {
                Thread.sleep(300);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            movement.toggleClaw(0.675);
            movement.toggleArm(0.98);

            movement.moveLift(-0.2);

            try {
                Thread.sleep(5900);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            movement.moveLift(0);

            movement.toggleClaw(1);
        }
    }
}
