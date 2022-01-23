package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.TelemLog;

public class Manipulator {
    public final Motor intake;
    public final Motor lift;
    public final Motor flyWheel;
    public final ServoMotor claw;
    public final ServoMotor arm;
    private final TelemLog telemetry;

    public Manipulator(HardwareMap hardwareMap, TelemLog telemetry) {
        this.intake = new Motor("Intake", hardwareMap, DcMotor.Direction.REVERSE);
        this.lift = new Motor("Lift", hardwareMap);
        this.flyWheel = new Motor("Flywheel", hardwareMap);
        this.claw = new ServoMotor("Claw", hardwareMap);
        this.arm = new ServoMotor("Arm", hardwareMap);

        this.telemetry = telemetry;
    }

    public void updatePositionReadings() {
        String intakeT = RobotHardware.DECIMAL_FORMAT.format(intake.getCurrentPosition());
        String liftT = RobotHardware.DECIMAL_FORMAT.format(lift.getCurrentPosition());
        String flyWheelT = RobotHardware.DECIMAL_FORMAT.format(flyWheel.getCurrentPosition());
        String clawT = RobotHardware.DECIMAL_FORMAT.format(claw.getCurrentPosition());
        String armT = RobotHardware.DECIMAL_FORMAT.format(arm.getCurrentPosition());

        telemetry.addData("Intake Position Centimeters : " , intakeT);
        telemetry.addData("Lift Encoder Position Centimeters : " , liftT);
        telemetry.addData("FlyWheel Encoder Position Centimeters : " , flyWheelT);
        telemetry.addData("Claw Encoder Position Centimeters : " , clawT);
        telemetry.addData("Arm Encoder Position Centimeters : " , armT);
    }
}
