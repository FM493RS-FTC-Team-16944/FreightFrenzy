package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class Manipulator {
    public final Motor intake;
    public final Motor lift;
    public final Motor flyWheel;
    public final ServoMotor claw;
    public final ServoMotor arm;
    private final Telemetry telemetry;

    public Manipulator(HardwareMap hardwareMap, Telemetry telemetry) {
        this.intake = new Motor("Intake", hardwareMap, DcMotor.Direction.REVERSE);
        this.lift = new Motor("Lift", hardwareMap);
        this.flyWheel = new Motor("Flywheel", hardwareMap);
        this.claw = new ServoMotor("Claw", hardwareMap);
        this.arm = new ServoMotor("Arm", hardwareMap);

        this.telemetry = telemetry;
    }

    public void outputPositionReadings() {
        telemetry.addData("Intake Position Centimeters : " , intake.getCurrentPosition());
        telemetry.addData("Lift Encoder Position Centimeters : " , lift.getCurrentPosition());
        telemetry.addData("FlyWheel Encoder Position Centimeters : " , flyWheel.getCurrentPosition());
        telemetry.addData("Claw Encoder Position Centimeters : " , claw.getCurrentPosition());
        telemetry.addData("Arm Encoder Position Centimeters : " , arm.getCurrentPosition());

        telemetry.update();
    }
}
