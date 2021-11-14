package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class armPid extends LinearOpMode {


    public static double angle = 0 //will be adjusted as driver moves arm
    private DcMotor arm;
    public static PIDCoefficients pidCos = new PIDCoefficients(0, 0, 0); //PID coefficients that need to be tuned
    public PIDCoefficients pidGs= new PIDCoefficients(0, 0, 0); //PID gains which we will define later in the process

    ElapsedTime PIDTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS); //timer

    @Override
    public void runOpMode() {
        /**initilization of motor
        arm.hardwareMap.get("needs to be filled in");

         resetting and starting encoders to track angle
         arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
**/

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                PID(angle); //running the PID algorithm at our target angle while the opMode is active

                telemetry.update();
            }
        }
    }

    double lastError = 0;
    double integral = 0;


    public void PID(double targetAngle){
        PIDTimer.reset();

        double currentAngle = arm.getCurrentPosition();
        double error = targetAngle - currentAngle;

        double changeInEr = error - lastError;
        double derivative = changeInEr / PIDTimer.time();

        integral += error * PIDTimer.time();

        pidGs.p = error * pidCos.p;
        pidGs.i = integral * pidCos.i;
        pidGs.d = derivative * pidCos.d;

        arm.setPower(pidGs.p + pidGs.i + pidGs.d);

        lastError = error;
    }
}