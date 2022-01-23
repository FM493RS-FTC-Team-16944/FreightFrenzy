package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.movement.RobotMovement;

public class Robot {
    public LinearOpMode teleOP;

    public RobotHardware hardware;
    public RobotMovement movement;
    public TelemLog telemLog;

    public Robot(LinearOpMode teleOP) {
        this.teleOP = teleOP;

        this.telemLog = new TelemLog(this.teleOP.telemetry);
        this.hardware = new RobotHardware(this);
        this.movement = new RobotMovement(this.hardware);
    }
}
