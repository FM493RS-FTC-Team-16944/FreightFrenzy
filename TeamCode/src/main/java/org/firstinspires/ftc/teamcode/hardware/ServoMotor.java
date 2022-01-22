package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ServoMotor {
    private final Servo servo;

    public ServoMotor(String name, HardwareMap hardwareMap){
        this.servo = hardwareMap.servo.get(name);
    }

    public void setPosition(double position) {
        this.servo.setPosition(position);
    }

    public double getCurrentPosition() {
        return this.servo.getPosition();
    }
}
