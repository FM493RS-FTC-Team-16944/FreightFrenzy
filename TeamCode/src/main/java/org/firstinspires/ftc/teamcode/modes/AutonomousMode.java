package org.firstinspires.ftc.teamcode.modes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.movement.Odometry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.models.XyhVector;

@Autonomous
public class AutonomousMode extends LinearOpMode {
    private Robot robot;

    public RobotHardware hardware;
    public Odometry odometry;
    public ObjectDetection recognition;

    public XyhVector warehouse;
    public XyhVector shippingHub;
    public XyhVector carousel;

    @Override
    public void runOpMode() {
        waitForStart();

        robot = new Robot(this);
        hardware = robot.hardware;
        odometry = hardware.odometry;
        recognition = new ObjectDetection(hardware);

        recognition.setupDetection();

        // THIS SHOULD BE TIMED LOOP
        while(opModeIsActive() && !isStopRequested()) {
            goToWarehouse();
            pickUpNearestFreight();

            goDropShippingHub();
        }

        // SPIN WHEEL, THIS SHOULD BE TIMED TOO
        getCarouselDucks();

        // LAND IN WAREHOUSE
        goToWarehouse();
    }

    public void goToPlace(XyhVector position) {
        // TODO: USE THIS FUNCTION FOR PID AND CALL IT FROM EVERY WEHRE ELSE
    }

    public void goToWarehouse() {
        XyhVector place = odometry.getPosition().subtract(warehouse);

        goToPlace(place);
    }

    public void pickUpNearestFreight() {
        if(!hardware.state.freightLoaded) {
            Recognition object = recognition.getNearestObject();

            // TODO: Im guessing this is the  distance from the robot to the object but pls test i beg tnx
            double x = object.getBottom();
            double y = object.getLeft();
            double h = 0.0; // TODO: FIX

            goToPlace(
                    new XyhVector(x, y, h)
            );

            // TODO: im pretty sure this moves it down but i forgot and am dumb pls check
            if(hardware.state.liftSpeed != 0.0) {
                robot.movement.toggleRaiseLift();
            }

            if(hardware.state.clawPosition == 1) {
                robot.movement.toggleClaw();
            }

            robot.movement.activateIntake();

            if(hardware.state.clawPosition == 0.675) {
                robot.movement.toggleClaw();
            }

            robot.movement.activateIntake();

            hardware.state.freightLoaded = true;
        }
    }

    public void goDropShippingHub() {
        XyhVector position = odometry.getPosition().subtract(shippingHub);

        goToPlace(position);

        if(hardware.state.liftSpeed != 0.5) {
            robot.movement.toggleRaiseLift();
        }

        if(hardware.state.clawPosition == 1) {
            robot.movement.toggleClaw();
        }
    }

    public void getCarouselDucks() {
        XyhVector position = odometry.getPosition().subtract(carousel);

        goToPlace(position);

        if(hardware.state.flyWheelSpeed == 0) {
            robot.movement.activateFlywheel();
        }

        // TODO: how 2 pick up duck, do we even need to i have no clue

        robot.movement.activateFlywheel();
    }
}
