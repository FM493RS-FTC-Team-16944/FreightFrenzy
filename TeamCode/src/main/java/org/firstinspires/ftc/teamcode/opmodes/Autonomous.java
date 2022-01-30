package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.GoToPosition;
import org.firstinspires.ftc.teamcode.LiftMacro;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.SequentialMovements;
import org.firstinspires.ftc.teamcode.models.Lift;
import org.firstinspires.ftc.teamcode.models.Task;
import org.firstinspires.ftc.teamcode.models.XyhVector;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous()
public class Autonomous extends LinearOpMode {
    public static final int THRESHOLD = 3;

    private Robot robot;
    public RobotHardware hardware;

    private GoToPosition goToCarousel;
    private GoToPosition rotateCarousel;
    private GoToPosition goToShippingHub;
    private GoToPosition goToShippingHubTop;
    private GoToPosition goToShippingHubMiddle;
    private GoToPosition goToShippingHubBottom;
    private GoToPosition unRotateCarousel;
    private GoToPosition warehouseA;
    private GoToPosition warehouseB;
    private long startSpin;
    public int j;
    public int spot;

    private SequentialMovements goToWarehouse;

    List<Task> tasks = new ArrayList<>();
    private boolean mvmt1, mvmt2, mvmt3, mvmt4, mvmt5, mvmt6, mvmt7;
    public long timestamp;

    @Override
    public void runOpMode() {
        tasks.add(arg -> spinCarousel((Long) arg));
        tasks.add(arg -> goDropShippingHub());
        tasks.add(arg -> goToWarehouse());
        spot = 2;

        robot = new Robot(this);
        hardware = robot.hardware;
        //hardware.pos = hardware.START_POS;

        robot.movement.toggleClaw(0.675);

        this.goToCarousel = new GoToPosition(robot, new XyhVector(14,65,0), this, false);
        this.rotateCarousel = new GoToPosition(robot, new XyhVector(14,65, Math.toRadians(40)), this, true);
        this.unRotateCarousel = new GoToPosition(robot, new XyhVector(14,65,Math.toRadians(0)), this, true);
        this.goToShippingHubTop = new GoToPosition(robot, new XyhVector(100,10,0), this, false);
        //this.goToShippingHubMiddle = new GoToPosition(robot, new XyhVector(100,20,0), this, false);  //TODO: WHAT PSOITONS
        //this.goToShippingHubBottom = new GoToPosition(robot, new XyhVector(100,30,0), this, false);
        this.warehouseA = new GoToPosition(robot, new XyhVector(5,-20,0), this, false);
        this.warehouseB = new GoToPosition(robot, new XyhVector(5,-192,0), this, false);

        flagsInit();
        robot.movement.activateFlywheel(0);

        waitForStart();
        long globalStart = System.currentTimeMillis();
        int i = 0;
        j = 0;

        while(opModeIsActive() && !isStopRequested()) {
            if (i==0){
                globalStart = System.currentTimeMillis();
                i++;
            }
            timestamp = System.currentTimeMillis() - globalStart;
            hardware.odometry();
            doNextTask(timestamp);
            telemetry.update();
            telemetry.addData("Time ", timestamp/1000);

        }
    }

    public boolean goToWarehouse() {
        telemetry.addLine("Warehouse");
        boolean warehouseA = this.warehouseA.runWithPID(THRESHOLD);

        if (warehouseA) {
            mvmt6 = true;
        }

        if (mvmt6) {
            telemetry.addLine("hi");
            boolean warehouseB = this.warehouseB.runWithPID(THRESHOLD);

            if (warehouseB) {
                //robot.movement.strafe(0,0,0);
                return true;
            }
        }

        return false;
    }

    public boolean goDropShippingHub() {
        telemetry.addLine("Shipping hub");
        long height = 2500;
        goToShippingHub = goToShippingHubTop;

        if (this.spot == 0) {
            height = 1500;
            goToShippingHub = goToShippingHubBottom;
        } else if (this.spot == 1) {
            height = 1500;
            goToShippingHub = goToShippingHubMiddle;
        }

        LiftMacro liftMacroUp = new LiftMacro(robot.movement, Lift.UP, height);
        Thread t1 = new Thread(liftMacroUp);
        t1.start();

        boolean finished = this.goToShippingHub.runWithPID(THRESHOLD);

        if (finished) {
            mvmt5 = true;
            telemetry.addData("Finished moving to hub", finished);
        }

        if(mvmt5) {
            telemetry.addLine("Hi");
            if (j==0){
                this.robot.movement.toggleClaw(1);
                j++;
            }

            LiftMacro liftMacroDown = new LiftMacro(robot.movement, Lift.DOWN, 2500);
            Thread t2 = new Thread(liftMacroDown);
            t2.start();

            return true;

            //return true;
        }

        return false;
    }

    public boolean spinCarousel(long start) {
        boolean finished = this.goToCarousel.runWithPID(THRESHOLD);
        if (finished) {
            mvmt1 = true;
        }

        if(mvmt1) {
            boolean finishedRotate = this.rotateCarousel.runWithPID(THRESHOLD);
            if (finishedRotate) {
                mvmt2 = true;
                telemetry.addLine("Rotated");
            }

            if(mvmt2) {
                if(hardware.flyWheelSpeed == 0) {
                    robot.movement.activateFlywheel(0.4);
                    telemetry.addLine("Spinning carousel");
                    startSpin = this.timestamp;
                }

                if(startSpin + 3000 < this.timestamp) {
                    robot.movement.activateFlywheel(0);
                    mvmt3 = true;
                }

                telemetry.addData("Mvmt3", mvmt3);

                if (mvmt3) {
                    telemetry.addLine("Unspinning Robot");

                    boolean finishedReturnCarousel = this.unRotateCarousel.runWithPID(THRESHOLD);

                    if (finishedReturnCarousel) {
                        mvmt4 = true;
                        telemetry.addLine("All done spinning");
                    }

                    if (mvmt4) {
                        return true;
                    }
                }
            }
        }

        return false;
    }

    public void doNextTask(long start) {
        for (Task task : this.tasks) {
            boolean done = task.run(start);

            if(done) {
                this.tasks.remove(0);
            }

            break;
        }
    }

    public void flagsInit(){
        this.mvmt1 = false;
        this.mvmt2 = false;
        this.mvmt3 = false;
        this.mvmt4 = false;
        this.mvmt5 = false;
        this.mvmt6 = false;
        this.mvmt7 = false;
    }
}
