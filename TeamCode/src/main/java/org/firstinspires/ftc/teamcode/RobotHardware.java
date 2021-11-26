package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class RobotHardware {

    private HardwareMap hardwareMap;
    DcMotor FrontRight, FrontLeft, BackRight, BackLeft, Intake, Lift, Flywheel, leftEncoder, rightEncoder, auxEncoder;
    Servo Carriage;

    public RobotHardware(HardwareMap aHardwareMap) {

        hardwareMap = aHardwareMap;

        FrontRight = hardwareMap.dcMotor.get("FrontRight");
        FrontRight.setDirection(DcMotor.Direction.FORWARD);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FrontLeft = hardwareMap.dcMotor.get("FrontLeft");
        FrontLeft.setDirection(DcMotor.Direction.FORWARD);
        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BackRight = hardwareMap.dcMotor.get("BackRight");
        BackRight.setDirection(DcMotor.Direction.FORWARD);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BackLeft = hardwareMap.dcMotor.get("BackLeft");
        BackLeft.setDirection(DcMotor.Direction.FORWARD);
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Intake = hardwareMap.dcMotor.get("Intake");
        Lift = hardwareMap.dcMotor.get("Lift");
        Flywheel = hardwareMap.dcMotor.get("Flywheel");
        Carriage = hardwareMap.servo.get("Carriage");
        Intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightEncoder = Lift;
        auxEncoder = Intake;
        leftEncoder = Flywheel;

        stop();
        resetDriveEncoders();

    }

    public void resetDriveEncoders() {
        Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void odometry(){
        while(opModeIsActive()){
            RobotHardware.updatePosition();

            telemetry.add
        }

    }


    private double L = 24.09;
    private double B = 10; //needs to be fixed
    private double R = 2.54;
    private double N = 8192;
    private double cm_per_tick = 2.0 * Math.PI * R / N;
    public XyhVector START_POS = new XyhVector();
    public XyhVector pos = new XyhVector(START_POS);

    private double previousRightPos = 0;
    private double previousLeftPos = 0;
    private double previousAuxPos = 0;

    public double currentRightPos = 0;
    public double currentLeftPos = 0;
    public double currentAuxPos = 0;

    public void updatePostition(){
        this.previousLeftPos = this.currentLeftPos;
        this.previousRightPos = this.currentRightPos;
        this.previousAuxPos = this.currentAuxPos;

        this.currentLeftPos = leftEncoder.getCurrentPosition();
        this.currentRightPos = rightEncoder.getCurrentPosition();
        this.currentAuxPos = auxEncoder.getCurrentPosition();

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

}

