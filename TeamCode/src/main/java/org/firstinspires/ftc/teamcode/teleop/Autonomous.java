package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
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
    private Robot robot;

    public RobotHardware hardware;

    List<Task> tasks = new ArrayList<>();

    @Override
    public void runOpMode() {
        // tasks.add(arg -> spinCarousel((Long) arg));
        tasks.add(arg -> goDropShippingHub());
        tasks.add(arg -> goToWarehouse());

        robot = new Robot(this);
        hardware = robot.hardware;

        robot.movement.toggleClaw(0.675);

        waitForStart();

        long timestamp = System.currentTimeMillis() / 1000;

        while(opModeIsActive() && !isStopRequested()) {
            doNextTask(timestamp);
        }
    }

    public boolean goToWarehouse() {
        int threshold = 3;

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

        SequentialMovements seqMovements = new SequentialMovements(waypoints, threshold, this);

        return seqMovements.runMovements();
    }

    public boolean goDropShippingHub() {
        int threshold = 3;
        XyhVector shippingHubPos = new XyhVector(82,17,0);
        GoToPosition goToShippingHub = new GoToPosition(robot, shippingHubPos, this);

        /*
        LiftMacro liftMacroUp = new LiftMacro(robot.movement, Lift.UP);
        Thread t1 = new Thread(liftMacroUp);
        t1.start();
         */

        boolean finished = goToShippingHub.runWithPID(threshold);

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
        int threshold = 3;
        XyhVector carousel = new XyhVector(14,63,0);
        GoToPosition goToCarousel = new GoToPosition(robot, carousel, this);

        boolean finished = goToCarousel.runWithPID(threshold);

        if(finished) {
            XyhVector rotatePos = new XyhVector(14,63, Math.toRadians(-32));
            GoToPosition gotoRotate = new GoToPosition(robot, rotatePos, this);

            boolean finishedRotate = gotoRotate.runWithPID(threshold);

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
