package org.firstinspires.ftc.teamcode;


public class RobotMovement {
    RobotHardware hardware;

    public int open = 0;

    RobotMovement(Robot robot) {
        this.hardware = robot.hardware;
    }

    public void strafe(double x, double y, double h) {

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(h), 1);
        double frontLeftPower = (y + x + h) / denominator;
        double backLeftPower = (y - x + h) / denominator;
        double frontRightPower = (y + x - h) / denominator;
        double backRightPower = (y - x - h) / denominator;

        hardware.frontLeftMotor.setPower(0.5*frontLeftPower);
        hardware.backLeftMotor.setPower(0.5*backLeftPower);
        hardware.frontRightMotor.setPower(0.5*frontRightPower);
        hardware.backRightMotor.setPower(0.5*backRightPower);
    }

    public void strafeR(double x, double y, double h) {
        double xR = -x * Math.cos(-this.hardware.pos.h) + y * Math.sin(-this.hardware.pos.h);
        double yR = -x * Math.sin(-this.hardware.pos.h) - y * Math.cos(-this.hardware.pos.h);

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(h), 1);

        double frontLeftPower = (yR + xR + h) / denominator;
        double backLeftPower = (yR - xR + h) / denominator;
        double frontRightPower = (yR + xR - h) / denominator;
        double backRightPower = (yR - xR - h) / denominator;

        hardware.frontLeftMotor.setPower(0.5*frontLeftPower);
        hardware.backLeftMotor.setPower(0.5*backLeftPower);
        hardware.frontRightMotor.setPower(0.5*frontRightPower);
        hardware.backRightMotor.setPower(0.5*backRightPower);
    }

    public void activateFlywheel(double speed) {
        hardware.flyWheelSpeed = speed;
        hardware.flywheel.setPower(speed);
    }

    public void activateFlywheel() {
        if(hardware.flyWheelSpeed == 0.5) {
            hardware.flyWheelSpeed = 0.0;
            hardware.flywheel.setPower(0.0);
        } else {
            hardware.flyWheelSpeed = 0.5;
            hardware.flywheel.setPower(0.5);
        }
    }

    public void activateIntake(double speed) {
        hardware.intakeSpeed = speed;
        hardware.intake.setPower(speed);
    }

    public void activateIntake() {
        if(hardware.intakeSpeed == 0.8) {
            hardware.intakeSpeed = 0.0;
            hardware.intake.setPower(0.0);
        } else {
            hardware.intakeSpeed = 0.8;
            hardware.intake.setPower(0.8);
        }
    }

    public void toggleRaiseLift() {
        if(open == 0) {
            if (hardware.liftSpeed != 0) {
                hardware.liftSpeed = 0.0;
                hardware.lift.setPower(0.0);
            } else {
                hardware.liftSpeed = 0.5;
                hardware.lift.setPower(0.5);
            }
        }
    }

    public void toggleLowerLift() {
        if(hardware.liftSpeed != 0) {
            hardware.liftSpeed = 0.0;
            hardware.lift.setPower(0.0);
        } else {
            hardware.liftSpeed = -0.2;
            hardware.lift.setPower(-0.2);
        }
    }

    public void moveLift(double Power) {
        //if (Power!=0){
            hardware.liftSpeed = Power;
            hardware.lift.setPower(Power);
        //}

    }


    public void toggleClaw() {
        // 1 is closed, 0.675 is opened

        if(hardware.clawPosition == 1) {
            hardware.clawPosition = 0.675;
            hardware.claw.setPosition(0.675);
        } else {
            hardware.clawPosition = 1;
            hardware.claw.setPosition(1);
        }
    }

    public void toggleClaw(double position) {
        hardware.claw.setPosition(position);
    }

    public void toggleArm() {
        // 1 is closed, 0.675 is opened

        if(hardware.armPosition == 0.98) {
            hardware.armPosition = 0.5;
            hardware.arm.setPosition(0.5);
            open = 1;
        } else {
            hardware.armPosition = 0.98;
            hardware.arm.setPosition(0.98);
            open = 0;
        }
    }

    public void toggleArm(double position) {
        hardware.arm.setPosition(position);
    }
}
