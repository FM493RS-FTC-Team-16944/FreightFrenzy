package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

public class Robot {
    public LinearOpMode teleOP;
    public RobotHardware hardware;
    public RobotMovement movement;

    public Robot(LinearOpMode teleOP) {
        this.teleOP = teleOP;

        this.hardware = new RobotHardware(this);
        this.movement = new RobotMovement(this.hardware);
    }
}
