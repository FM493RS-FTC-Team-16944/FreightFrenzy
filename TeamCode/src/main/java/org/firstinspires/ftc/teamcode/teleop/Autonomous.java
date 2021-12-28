package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.XyhVector;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous()
public class Autonomous extends LinearOpMode {
    private Robot robot;

    public RobotHardware hardware;
    public ObjectDetection recognition;

    public XyhVector warehouse;
    public XyhVector shippingHub;
    public XyhVector carousel;

    @Override
    public void runOpMode() {
        waitForStart();

        robot = new Robot(this);
        hardware = robot.hardware;
        recognition = new ObjectDetection(hardware);

        recognition.setupDetection();

        // THIS SHOULD BE TIMED LOOP
        while(opModeIsActive() && !isStopRequested()) {
            goToWarehouse();
            pickUpNearestFreight();

            goDropShippingHub();
        }

        // SPIN WHEEL, THIS SHUD BE TIMED TOO
        getCarouselDucks();

        // LAND IN WAREHOUSE
        goToWarehouse();
    }

    public void goToPlace(double x, double y, double h) {
        // TODO: USE THIS FUNCTION FOR PID AND CALL IT FROM EVERY WEHRE ELSE
    }

    public void goToWarehouse() {
        double x = hardware.pos.x - warehouse.x;
        double y = hardware.pos.y - warehouse.y; // TODO: im pretty sure this isnt negative but uh YEP
        double h = hardware.pos.h - warehouse.h;

        goToPlace(x, y, h);
    }

    public void pickUpNearestFreight() {
        if(!hardware.freightLoaded) {
            Recognition object = recognition.getNearestObject();

            // TODO: Im guessing this is the  distance from the robot to the object but pls test i beg tnx
            double x = object.getBottom();
            double y = object.getLeft();
            double h = 0.0; // TODO: FIX

            goToPlace(x, y, h);

            // TODO: im pretty sure this moves it down but i forgot and am dumb pls check
            if(hardware.liftSpeed != 0.0) {
                robot.movement.toggleRaiseLift();
            }

            if(hardware.clawPosition == 1) {
                robot.movement.toggleClaw();
            }

            robot.movement.activateIntake();

            if(hardware.clawPosition == 0.675) {
                robot.movement.toggleClaw();
            }

            robot.movement.activateIntake();

            hardware.freightLoaded = true;
        }
    }

    public void goDropShippingHub() {
        double x = hardware.pos.x - shippingHub.x;
        double y = hardware.pos.y - shippingHub.y;
        double h = hardware.pos.h - shippingHub.h;

        goToPlace(x, y, h);

        if(hardware.liftSpeed != 0.5) {
            robot.movement.toggleRaiseLift();
        }

        if(hardware.clawPosition == 1) {
            robot.movement.toggleClaw();
        }
    }

    public void getCarouselDucks() {
        double x = hardware.pos.x - carousel.x;
        double y = hardware.pos.y - carousel.y;
        double h = hardware.pos.h - carousel.h;

        goToPlace(x, y, h);

        if(hardware.flyWheelSpeed == 0) {
            robot.movement.activateFlywheel();
        }

        // TODO: how 2 pick up duck, do we even need to i have no clue

        robot.movement.activateFlywheel();
    }
}
