package org.firstinspires.ftc.teamcode.movement;

import org.firstinspires.ftc.teamcode.hardware.Manipulator;
import org.firstinspires.ftc.teamcode.hardware.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

public class RobotMovement {
    private final RobotHardware.State state;
    private final MecanumDriveTrain driveTrain;
    private final Manipulator manipulator;

    public RobotMovement(RobotHardware robotHardware) {
        this.driveTrain = robotHardware.driveTrain;
        this.manipulator = robotHardware.manipulator;

        this.state = robotHardware.state;
    }

    public void strafe(double x, double y, double h) {
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(h), 1);

        double frontLeftPower = (y + x + h) / denominator;
        double backLeftPower = (y - x + h) / denominator;
        double frontRightPower = (y + x - h) / denominator;
        double backRightPower = (y - x - h) / denominator;

        driveTrain.topLeft.setPower(0.5 * frontLeftPower);
        driveTrain.backLeft.setPower(0.5 * backLeftPower);
        driveTrain.topRight.setPower(0.5 * frontRightPower);
        driveTrain.backRight.setPower(0.5 * backRightPower);
    }

    public void activateFlywheel(double speed) {
        this.state.flyWheelSpeed = speed;

        manipulator.flyWheel.setPower(speed);
    }

    public void activateFlywheel() {
        if(this.state.flyWheelSpeed == 1) {
            this.state.flyWheelSpeed = 0.0;
        } else {
            this.state.flyWheelSpeed = 1.0;
        }

        manipulator.flyWheel.setPower(this.state.flyWheelSpeed);
    }

    public void activateIntake(double speed) {
        this.state.intakeSpeed = speed;

        manipulator.intake.setPower(speed);
    }

    public void activateIntake() {
        if(this.state.intakeSpeed == 1) {
            this.state.intakeSpeed = 0.0;
        } else {
            this.state.intakeSpeed = 1.0;
        }

        this.manipulator.intake.setPower(1.0);
    }

    public void toggleRaiseLift() {
        if(!this.state.armOpen) {
            if (this.state.liftSpeed == 0.5) {
                this.state.liftSpeed = 0.0;
            } else {
                this.state.liftSpeed = 0.5;
            }

            this.manipulator.lift.setPower(this.state.liftSpeed);
        }
    }

    public void toggleLowerLift() {
        if(this.state.liftSpeed == -0.2) {
            this.state.liftSpeed = 0.0;
        } else {
            this.state.liftSpeed = -0.2;
        }

        this.manipulator.lift.setPower(this.state.liftSpeed);
    }


    public void toggleClaw() {
        // 1 is closed, 0.675 is opened

        if(this.state.clawPosition == 1) {
            this.state.clawPosition = 0.675;
        } else {
            this.state.clawPosition = 1;
        }

        this.manipulator.claw.setPosition(0.675);
    }

    public void toggleClaw(double position) {
        this.manipulator.claw.setPosition(position);
    }

    public void toggleArm() {
        // 1 is closed, 0.675 is opened

        if(this.state.armPosition == 0.98) {
            this.state.armPosition = 0.5;
            this.state.armOpen = true;
        } else {
            this.state.armPosition = 0.98;
            this.state.armOpen = false;
        }

        this.manipulator.arm.setPosition(0.5);
    }

    public void toggleArm(double position) {
        this.manipulator.arm.setPosition(position);
    }
}
