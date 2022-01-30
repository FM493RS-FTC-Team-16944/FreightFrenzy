package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.models.Lift;

public class LiftMacro implements Runnable {
    RobotMovement movement;
    Lift liftUpDown;
    public boolean complete;
    long height;

    public LiftMacro(RobotMovement movement, Lift liftUpDown, long height) {
        this.movement = movement;
        this.height = height;
        this.liftUpDown = liftUpDown;
    }       

    @Override
    public void run() {
        if(liftUpDown == Lift.UP) {
            movement.activateIntake(-0.8);
            movement.moveLift(0.5);

            try {
                Thread.sleep(height);         //2500 is max height
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            movement.activateIntake(0);

            movement.moveLift(0);
            if (height > 2000) {
                movement.toggleArm(0.65 );
            }

            this.complete = true;
        } else {
            movement.toggleClaw(1);
            try {
                Thread.sleep(1000);
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
