package org.firstinspires.ftc.teamcode;

public class RobotRelativeDeltas {
    public void runOpMode(){


        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()){

                telemetry.update();
            }
        }
    }
}
