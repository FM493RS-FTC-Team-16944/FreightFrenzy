package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class Robot {
    public LinearOpMode teleOP;
    public RobotHardware hardware;
    public RobotMovement movement;

    public Robot(LinearOpMode teleOP) {
        this.teleOP = teleOP;

        this.hardware = new RobotHardware(this);
        this.movement = new RobotMovement(this);
    }
}
