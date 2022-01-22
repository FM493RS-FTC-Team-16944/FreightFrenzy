package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.TelemLog;

public class MecanumDriveTrain {
    public final Motor topLeft;
    public final Motor backLeft;
    public final Motor topRight;
    public final Motor backRight;
    public final Motor leftEncoder;
    public final Motor rightEncoder;
    public final Motor auxEncoder;
    private final TelemLog telemetry;

    public MecanumDriveTrain(
            String topLeftName,
            String backLeftName,
            String topRightName,
            String backRightName,
            String leftEncoderName,
            String rightEncoderName,
            String auxEncoderName,
            HardwareMap hardwareMap,
            TelemLog telemetry
    ) {
        this.topRight = new Motor(topRightName, hardwareMap, DcMotor.Direction.FORWARD);
        this.topLeft = new Motor(topLeftName, hardwareMap, DcMotor.Direction.REVERSE);
        this.backLeft = new Motor(backLeftName, hardwareMap, DcMotor.Direction.REVERSE);
        this.backRight = new Motor(backRightName, hardwareMap, DcMotor.Direction.FORWARD);

        this.leftEncoder = new Motor(leftEncoderName, hardwareMap);
        this.rightEncoder = new Motor(rightEncoderName, hardwareMap, DcMotor.Direction.REVERSE);
        this.auxEncoder = new Motor(auxEncoderName, hardwareMap, DcMotor.Direction.REVERSE);

        // You can call the methods to set the motor modes
        this.leftEncoder.reset();
        this.rightEncoder.reset();
        this.auxEncoder.reset();

        this.stop();

        // This part is depends on your preference for zero power behavior. For the implementation on my robot, I will set it to break mode.
        this.topLeft.setBreakMode();
        this.backLeft.setBreakMode();
        this.topRight.setBreakMode();
        this.backRight.setBreakMode();

        this.telemetry = telemetry;
    }

    public void stop() {
        this.topRight.setPower(0);
        this.topLeft.setPower(0);
        this.backRight.setPower(0);
        this.backLeft.setPower(0);
    }

    public void outputEncoderReadings() {
        telemetry.addData("Left Encoder Position Centimeters : " , leftEncoder.getCurrentPosition());
        telemetry.addData("Right Encoder Position Centimeters : " , rightEncoder.getCurrentPosition());
        telemetry.addData("Auxiliary Encoder Position Centimeters : " , auxEncoder.getCurrentPosition());

        telemetry.update();
    }
}
