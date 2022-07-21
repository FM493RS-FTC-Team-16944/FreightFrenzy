package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Rotation;
import org.firstinspires.ftc.teamcode.models.Mode;
import org.firstinspires.ftc.teamcode.models.Translation2d;
import org.firstinspires.ftc.teamcode.models.XyhVector;
import org.firstinspires.ftc.teamcode.models.Twist2d;
import org.firstinspires.ftc.teamcode.models.Rotation2d;
import org.firstinspires.ftc.teamcode.models.Transform2d;
import org.firstinspires.ftc.teamcode.models.Translation2d;
import org.firstinspires.ftc.teamcode.models.Pose2d;

public class RobotHardware {

    public DcMotor frontRightMotor, frontLeftMotor, backRightMotor, backLeftMotor;
    public DcMotor intake, lift, flywheel;
    public DcMotor leftEncoder, rightEncoder, auxEncoder;

    public Servo claw, arm;

    public HardwareMap hardwareMap;
    public Mode currentMode = Mode.DRIVER_CONTROL;

    public double intakeSpeed = 0.0;
    public double flyWheelSpeed = 0.0;
    public double liftSpeed = 0.0;
    public double clawPosition = 0.0;
    public double armPosition = 0.0;

    public Pose2d robotPose = new Pose2d();

    public boolean freightLoaded = true;

    public XyhVector START_POS = new XyhVector(0, 0, 0);    //Robot starts at 0,0,0 coord (Can be adjusted later)
    public XyhVector pos = new XyhVector(START_POS);

    public double currentRightPos = 0;
    public double currentLeftPos = 0;
    public double currentAuxPos = 0;

    public BNO055IMU imu;
    Orientation lastAngles = new Orientation();

    private double auxWidth = 11;                              //Robot Geometry for odom
    private double trackWidth = 29;                                 //needs to be remeasured
    private double R = 2.54;
    private double N = 8192;
    private double cm_per_tick = 2.0 * Math.PI * R / N;
    private double previousRightPos = 0;
    private double previousLeftPos = 0;
    private double previousAuxPos = 0;
    private Rotation2d previousAngle = new Rotation2d(0);
    public double globalAngle = 0;
    public double dx;
    public double dy;
    public double deltaLeft;
    public double deltaRight;
    public double deltaHorizontal;

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
        lift.setDirection(DcMotor.Direction.REVERSE);
        flywheel = hardwareMap.dcMotor.get("Flywheel");
        claw = hardwareMap.servo.get("Claw");
        arm = hardwareMap.servo.get("Arm");

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        imu.initialize(parameters);

        resetAngle();

        setEncodersMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightEncoder = flywheel;
        auxEncoder = frontLeftMotor;
        leftEncoder = backLeftMotor;

        stop();
        resetDriveEncoders();

        while (!robot.teleOP.isStopRequested() && !imu.isGyroCalibrated()) {
            robot.teleOP.sleep(50);
            robot.teleOP.idle();
        }
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

    //TODO: IMU FIXES & ODOM

    public void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

        globalAngle = 0;
    }


    public void odometry() {



        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

        double deltaAngle = -(angles.firstAngle - lastAngles.firstAngle);

        if (globalAngle < -Math.PI)
            globalAngle += 2 * Math.PI;
        else if (globalAngle > Math.PI)
            globalAngle -= 2 * Math.PI;

        globalAngle += deltaAngle;
        lastAngles = angles;


        double leftPosition = this.leftEncoder.getCurrentPosition() * cm_per_tick; //due to poor mapping
        double rightPosition = this.rightEncoder.getCurrentPosition() * cm_per_tick;
        double horizontalPosition = - this.auxEncoder.getCurrentPosition() *cm_per_tick;



        double deltaLeft = leftPosition - previousLeftPos;
        this.previousLeftPos = leftPosition; // Stores the current leftPosition to be used during the next update
        double deltaRight = rightPosition - previousRightPos;
        this.previousRightPos = rightPosition; // Stores the current rightPosition to be used during the next update
        double deltaHorizontal = horizontalPosition - previousAuxPos;
        this.previousAuxPos = horizontalPosition; // Stores the current horizontalPosition to be used during the next update

        Rotation2d angle = previousAngle.plus(
                new Rotation2d(
                        (deltaLeft - deltaRight) / trackWidth
                )
        );

        double dw = (angle.minus(previousAngle).getRadians());

        double dx = (deltaLeft + deltaRight) / 2;
        double dy = deltaHorizontal - (auxWidth * dw); //check this, supossed to be centerWheelOffset

        Twist2d twist2d = new Twist2d(dx,dy,dw);

        Pose2d newPose = robotPose.exp(twist2d);

        previousAngle = angle;

        robotPose = new Pose2d(newPose.getTranslation(), angle);

        //double deltaHeading = (-deltaLeft + deltaRight) / trackWidth;

//        double horizontalOffset = auxWidth * deltaAngle;
//        double relativeX = deltaHorizontal - horizontalOffset;
//        double relativeY = (-deltaLeft + deltaRight) / 2.0;
//
//        pos.x += Math.cos(globalAngle) * relativeX - Math.sin(globalAngle) * relativeY;
//        pos.y -= Math.sin(globalAngle) * relativeX + Math.cos(globalAngle) * relativeY;
//        pos.h = globalAngle;
    }

    public Pose2d exp(Twist2d twist){
        double dx = twist.dx;
        double dy = twist.dy;
        double dtheta = twist.dtheta;

        double sinTheta = Math.sin(dtheta);
        double cosTheta = Math.cos(dtheta);

        double s;
        double c;

        if (Math.abs(dtheta) < 1E-9){
            s = 1.0 - 1.0 / 6.0 * dtheta * dtheta;
            c = 0.5 * dtheta;
        }else{
            s = sinTheta / dtheta;
            c = (1 - cosTheta) / dtheta;
        }

        Transform2d transform = new Transform2d(new Translation2d(dx * s - dy * c, dx * c + dy * s),
                new Rotation2d(cosTheta,sinTheta));

        return robotPose.plus(transform);
    }
}

