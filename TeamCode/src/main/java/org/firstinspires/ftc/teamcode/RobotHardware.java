package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.models.Mode;

public class RobotHardware {
    public XyhVector START_POS = new XyhVector(0, 0, 0);    //Robot starts at 0,0,0 coord (Can be adjusted later)
    public XyhVector pos = new XyhVector(START_POS);

    public double currentRightPos = 0;
    public double currentLeftPos = 0;
    public double currentAuxPos = 0;

    public DcMotor frontRightMotor, frontLeftMotor, backRightMotor, backLeftMotor;
    public DcMotor intake, lift, flywheel;
    public DcMotor leftEncoder, rightEncoder, auxEncoder;

    public Servo carriage;

    public HardwareMap hardwareMap;
    public Mode currentMode = Mode.DRIVER_CONTROL;

    public double intakeSpeed = 0.0;
    public double flyWheelSpeed = 0.0;

    class LiftPositions {
        final double downPosition = 0.0;
        final double upPosition = 1.0;
    }

    private double L = 24.09;                              //Robot Geometry for odom
    private double B = 10;                                 //needs to be remeasured
    private double R = 2.54;
    private double N = 8192;
    private double cm_per_tick = 2.0 * Math.PI * R / N;
    private double previousRightPos = 0;
    private double previousLeftPos = 0;
    private double previousAuxPos = 0;
    /**
     * ...........................................................................................
     * ........................................HARDWARE...........................................
     * ...........................................................................................
     */

    public RobotHardware(HardwareMap aHardwareMap) {
        hardwareMap = aHardwareMap;

        frontRightMotor = hardwareMap.dcMotor.get("FrontRight");
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeftMotor = hardwareMap.dcMotor.get("FrontLeft");
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);   //So that all motors forward goes forward
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backRightMotor = hardwareMap.dcMotor.get("BackRight");
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backLeftMotor = hardwareMap.dcMotor.get("BackLeft");      //So that all motors forward goes forward
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake = hardwareMap.dcMotor.get("Intake");
        intake.setDirection(DcMotor.Direction.REVERSE);

        lift = hardwareMap.dcMotor.get("Lift");
        flywheel = hardwareMap.dcMotor.get("Flywheel");
        carriage = hardwareMap.servo.get("Carriage");

        setEncodersMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightEncoder = lift;
        auxEncoder = intake;
        leftEncoder = flywheel;

        stop();
        resetDriveEncoders();
    }

    public void resetDriveEncoders() {
        setEncodersMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setEncodersMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setEncodersMode(DcMotor.RunMode mode) {
        lift.setMode(mode);
        intake.setMode(mode);
        flywheel.setMode(mode);
    }

    public void stop() {
        frontRightMotor.setPower(0);
        frontLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        backLeftMotor.setPower(0);
    }


    /**
     * ...........................................................................................
     * ........................................ODOMETRY...........................................
     * ...........................................................................................
     */


    public void odometry() {
        this.previousRightPos = this.currentRightPos;
        this.previousLeftPos = this.currentLeftPos;
        this.previousAuxPos = this.currentAuxPos;

        this.currentRightPos = this.rightEncoder.getCurrentPosition(); //TODO: Determine if there should be + or -
        this.currentLeftPos = this.leftEncoder.getCurrentPosition();
        this.currentAuxPos = this.auxEncoder.getCurrentPosition();

        double deltaLeft = this.currentLeftPos - this.previousLeftPos;
        double deltaRight = this.currentRightPos - this.previousRightPos;
        double deltaAux = this.currentAuxPos - this.previousAuxPos;

        double deltaT = cm_per_tick * (deltaRight - deltaLeft);
        double dx = cm_per_tick * (deltaLeft + deltaRight);
        double dy = cm_per_tick * (deltaAux - (deltaRight - deltaLeft) * B / L);
        double theta = pos.h + (deltaT / 2.0);

        pos.x += dx * Math.cos(theta) - dy * Math.sin(theta);
        pos.y += dx * Math.sin(theta) - dy * Math.cos(theta);
        pos.h += deltaT;
    }
}