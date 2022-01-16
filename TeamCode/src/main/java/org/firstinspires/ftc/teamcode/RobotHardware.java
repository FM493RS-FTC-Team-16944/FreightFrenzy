package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.models.Mode;
import org.firstinspires.ftc.teamcode.models.XyhVector;

public class RobotHardware {
    public XyhVector START_POS = new XyhVector(0, 0, 0);    //Robot starts at 0,0,0 coord (Can be adjusted later)
    public XyhVector pos = new XyhVector(START_POS);

    public double currentRightPos = 0;
    public double currentLeftPos = 0;
    public double currentAuxPos = 0;

    public DcMotor frontRightMotor, frontLeftMotor, backRightMotor, backLeftMotor;
    public DcMotor intake, lift, flywheel;
    public DcMotor leftEncoder, rightEncoder, auxEncoder;

    public BNO055IMU imu;

    public Servo claw, arm;

    public HardwareMap hardwareMap;
    public Mode currentMode = Mode.DRIVER_CONTROL;

    public double intakeSpeed = 0.0;
    public double flyWheelSpeed = 0.0;
    public double liftSpeed = 0.0;
    public double clawPosition = 0.0;
    public double armPosition = 0.0;

    public boolean freightLoaded = true;

    public int angle = 0;

    private double L = 29;                              //Robot Geometry for odom
    private double B = 11;                                 //needs to be remeasured
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

    public RobotHardware(Robot robot) {
        hardwareMap = robot.teleOP.hardwareMap;

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
        claw = hardwareMap.servo.get("Claw");
        arm = hardwareMap.servo.get("Arm");

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        imu.initialize(parameters);

        setEncodersMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightEncoder = flywheel;
        auxEncoder = backLeftMotor;
        leftEncoder = frontLeftMotor;

        stop();
        resetDriveEncoders();
    }

    public void resetDriveEncoders() {
        setEncodersMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setEncodersMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setEncodersMode(DcMotor.RunMode mode) {
        flywheel.setMode(mode);
        backLeftMotor.setMode(mode);
        backRightMotor.setMode(mode);
        frontLeftMotor.setMode(mode);
        frontRightMotor.setMode(mode);
    }

    public void stop() {
        frontRightMotor.setPower(0);
        frontLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        backLeftMotor.setPower(0);
    }


    /**
     * ...........................................................................................
     * ........................................MECANUM ODOMETRY...........................................
     * ...........................................................................................
     */

    public void odometry() {
        this.previousRightPos = this.currentRightPos;
        this.previousLeftPos = this.currentLeftPos;
        this.previousAuxPos = this.currentAuxPos;

        this.currentRightPos = this.rightEncoder.getCurrentPosition(); //TODO: Determine if there should be + or -
        this.currentLeftPos = -this.leftEncoder.getCurrentPosition();
        this.currentAuxPos = -this.auxEncoder.getCurrentPosition();

        double deltaLeft = this.currentLeftPos - this.previousLeftPos;
        double deltaRight = this.currentRightPos - this.previousRightPos;
        double deltaAux = this.currentAuxPos - this.previousAuxPos;

        double deltaT = cm_per_tick * (deltaRight - deltaLeft) / L;
        double dx = cm_per_tick * (deltaLeft + deltaRight) / 2.0;
        double dy = cm_per_tick * (deltaAux - (deltaRight - deltaLeft) * B / L);
        double theta = pos.h + (deltaT / 2.0);

        pos.y += dx * Math.cos(theta) - dy * Math.sin(theta);
        pos.x += dx * Math.sin(theta) + dy * Math.cos(theta);
        pos.h += deltaT;
        pos.h %= 2*Math.PI;
    }
}