package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

public class posVolCtrlFF {
    private double destination = 0; //will be adjusted when method is called
    public static double targetVelocity = 0; // also needs to be fixed
    public static double targetAccel = 0; // needs to be fixed
    private DcMotor motor;

    //need to insert more motor variable initialization later
    public static double pidVol = 0; //PID coefficients that need to be tuned
    public static double pidAccel = 0;  //PID coefficients that need to be tuned


    ElapsedTime PIDTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS); //timer

    public void runOpMode() {
        /**initilization of motors
         arm.hardwareMap.get("needs to be filled in");

         resetting and starting encoders to track angle
         motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         **/
        this.destination = 0; // input will replace 0
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                PID(destination, targetVelocity, targetAccel); //running the PID algorithm at our target angle while the opMode is active

                telemetry.update();
            }
        }
    }

    double previousPosition = 0;//starting point
    double previousVelocity = 0;
    double previousAccel = 0;

    public void PID(double targetPosition, double targetVelocity, double targetAccel){

        double currentPosition = 0; //call localization function
        double currentVelocity = (currentPosition - previousPosition)/(PIDTimer.time());
        double velocityErr = targetVelocity - currentVelocity;

        double currentAccel = (currentVelocity - previousVelocity)/(PIDTimer.time());
        double accelErr = targetAccel - currentAccel;

        double outputControl = pidVol * velocityErr + pidAccel * accelErr;


        //needs to be altered to consider all four motors
        motor.setPower(outputControl);

        previousPosition = currentPosition;
        previousVelocity = currentVelocity;
        previousAccel = currentAccel;
        PIDTimer.reset();

    }
}
