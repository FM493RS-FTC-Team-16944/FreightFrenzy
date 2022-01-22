package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Motor {
    private static final double R = 2.54;
    private static final double N = 8192;
    private static final double CM_PER_TICK = 2.0 * Math.PI * R / N;

    private final DcMotor motor;

    public Motor(String name, HardwareMap hardwareMap){
        this.motor = hardwareMap.dcMotor.get(name);
    }

    public Motor(String name, HardwareMap hardwareMap, DcMotor.Direction direction){
        this.motor = hardwareMap.dcMotor.get(name);

        setupMotor(direction);
    }

    private void setupMotor(DcMotor.Direction direction) {
        this.motor.setDirection(direction);
        this.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void reset() {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setBreakMode() {
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setFloatMode() {
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void setPower(double power) {
        motor.setPower(power);
    }

    public double getCurrentPosition(){
        return motor.getCurrentPosition() * CM_PER_TICK;
    }
}
