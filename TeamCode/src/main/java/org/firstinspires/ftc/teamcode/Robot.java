package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.teleop.TeleOP;

public class Robot {
    public TeleOP teleOP;
    public RobotHardware hardware;
    public DriveMovement movement;

    public Robot(TeleOP teleOP) {
        this.teleOP = teleOP;
        this.hardware = teleOP.hardware;

        this.movement = new DriveMovement(this);
    }
}
