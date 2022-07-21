package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.GamePad;
import org.firstinspires.ftc.teamcode.GoToPosition;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.SequentialMovements;
import org.firstinspires.ftc.teamcode.models.XyhVector;

import java.util.LinkedHashMap;

@TeleOp(name = "TeleOp")
public class TeleOP extends LinearOpMode {
    public Robot robot;

    public RobotHardware hardware;
    public GamePad gamepad;

    @Override
    public void runOpMode() {
        waitForStart();

        robot = new Robot(this);
        hardware = robot.hardware;
        gamepad = new GamePad(robot, gamepad1);

        /*
        XyhVector targetVector = new XyhVector(10, 0, Math.toRadians(0));
        GoToPosition runToTarget = new GoToPosition(robot, targetVector, this);

        XyhVector thirdTarget = new XyhVector(10, 0, Math.toRadians(90));
        GoToPosition thirdPos = new GoToPosition(robot, thirdTarget, this);

        XyhVector lastTarget = new XyhVector(0, 0, Math.toRadians(0));
        GoToPosition backToOrigin = new GoToPosition(robot, lastTarget, this);

        LinkedHashMap<GoToPosition, Boolean> waypoints = new LinkedHashMap<>();
        waypoints.put(runToTarget, false);
        waypoints.put(thirdPos, false);
        //waypoints.put(backToOrigin, false);

        SequentialMovements path = new SequentialMovements(waypoints, 3, this);

        int navigator = 0;

         */

        hardware.resetDriveEncoders();

        while (opModeIsActive() && !isStopRequested()) {
            hardware.odometry();
            gamepad.updateRobot();

            //path.runMovements();

            telemetry.addData("Position X", hardware.robotPose.getX());
            telemetry.addData("Position Y", hardware.robotPose.getY());
            telemetry.addData("Posiion H", Math.toDegrees(hardware.robotPose.getHeading()));

            // telemetry.addData("Claw Position", hardware.claw.getPosition());
            // telemetry.addData("Arm Position", hardware.arm.getPosition());

            /**ODOM DEBUG **/
            // telemetry.addData("RightEncoder", hardware.currentRightPos);
            // telemetry.addData("LeftEncoder", hardware.currentLeftPos);
            // telemetry.addData("AuxEncoder", hardware.currentAuxPos);
            // telemetry.addData("IMU Angle", hardware.globalAngle);
            // telemetry.addData("RawLeft", hardware.leftEncoder.getCurrentPosition());
            // telemetry.addData("RawRight", hardware.rightEncoder.getCurrentPosition());
            // telemetry.addData("RawHori", hardware.auxEncoder.getCurrentPosition());


            telemetry.update();
        }
    }


}