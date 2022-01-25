package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.GoToPosition;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.SequentialMovements;
import org.firstinspires.ftc.teamcode.models.Task;
import org.firstinspires.ftc.teamcode.models.XyhVector;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous()
public class Autonomous extends LinearOpMode {
    private static final int THRESHOLD = 3;

    private Robot robot;
    public RobotHardware hardware;

    private GoToPosition goToCarousel;
    private GoToPosition rotateCarousel;
    private GoToPosition goToShippingHub;

    private SequentialMovements goToWarehouse;

    List<Task> tasks = new ArrayList<>();

    @Override
    public void runOpMode() {
        tasks.add(arg -> spinCarousel((Long) arg));
        tasks.add(arg -> goDropShippingHub());
        tasks.add(arg -> goToWarehouse());

        robot = new Robot(this);
        hardware = robot.hardware;

        robot.movement.toggleClaw(0.675);

        this.goToCarousel = new GoToPosition(robot, new XyhVector(14,63,0), this);
        this.rotateCarousel = new GoToPosition(robot, new XyhVector(14,63, Math.toRadians(-32)), this);
        this.goToShippingHub = new GoToPosition(robot, new XyhVector(82,17,0), this);

        XyhVector crossUp = new XyhVector(13,-54, Math.toRadians(0));
        GoToPosition goToCrossUp = new GoToPosition(robot, crossUp, this);

        XyhVector warehouse = new XyhVector(39,-196, Math.toRadians(0));
        GoToPosition goToWarehouse = new GoToPosition(robot, warehouse, this);

        XyhVector rotate = new XyhVector(39,-196, Math.toRadians(180));
        GoToPosition goToRotate = new GoToPosition(robot, rotate, this);

        LinkedHashMap<GoToPosition, Boolean> waypoints = new LinkedHashMap<>();
        waypoints.put(goToCrossUp, false);
        waypoints.put(goToWarehouse, false);
        waypoints.put(goToRotate, false);

        this.goToWarehouse = new SequentialMovements(waypoints, THRESHOLD, this);

        waitForStart();

        long timestamp = System.currentTimeMillis() / 1000;

        while(opModeIsActive() && !isStopRequested()) {
            doNextTask(timestamp);
        }
    }

    public boolean goToWarehouse() {
        return this.goToWarehouse.runMovements();
    }

    public boolean goDropShippingHub() {
        /*
        LiftMacro liftMacroUp = new LiftMacro(robot.movement, Lift.UP);
        Thread t1 = new Thread(liftMacroUp);
        t1.start();
         */

        boolean finished = this.goToShippingHub.runWithPID(THRESHOLD);

        if(finished /* && !t1.isAlive() */) {
            /*
            LiftMacro liftMacroDown = new LiftMacro(robot.movement, Lift.DOWN);
            Thread t2 = new Thread(liftMacroDown);
            t2.start();

            return !t2.isAlive();
             */
            return true;
        }

        return false;
    }

    public boolean spinCarousel(long start) {
        boolean finished = this.goToCarousel.runWithPID(THRESHOLD);


        if(finished) {
            boolean finishedRotate = this.rotateCarousel.runWithPID(THRESHOLD);

            if(finishedRotate) {
                if(hardware.flyWheelSpeed != 1) {
                    robot.movement.activateFlywheel(1);
                    return false;
                }

                long timestamp = System.currentTimeMillis() / 1000;

                if(start + 5 == timestamp) {
                    robot.movement.activateFlywheel(0);
                    return true;
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
}
