package org.firstinspires.ftc.teamcode;


public class RobotMovement {
    RobotHardware hardware;

    RobotMovement(Robot robot) {
        this.hardware = robot.hardware;
    }

    public void strafe(double x, double y, double h) {
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(h), 1);
        double frontLeftPower = (y   + x + h) / denominator;
        double backLeftPower = (x - y + h) / denominator;
        double frontRightPower = (x - y - h) / denominator;
        double backRightPower = (x + y - h) / denominator;

        hardware.frontLeftMotor.setPower(frontLeftPower);
        hardware.backLeftMotor.setPower(backLeftPower);
        hardware.frontRightMotor.setPower(frontRightPower);
        hardware.backRightMotor.setPower(backRightPower);
    }

    public void activateFlywheel(double speed) {
        hardware.flyWheelSpeed = speed;
        hardware.flywheel.setPower(speed);
    }

    public void activateFlywheel() {
        if(hardware.flyWheelSpeed == 1) {
            hardware.flyWheelSpeed = 0.0;
            hardware.flywheel.setPower(0.0);
        } else {
            hardware.flyWheelSpeed = 1.0;
            hardware.flywheel.setPower(1.0);
        }
    }

    public void activateIntake(double speed) {
        hardware.intakeSpeed = speed;
        hardware.intake.setPower(speed);
    }

    public void activateIntake() {
        if(hardware.intakeSpeed == 1) {
            hardware.intakeSpeed = 0.0;
            hardware.intake.setPower(0.0);
        } else {
            hardware.intakeSpeed = 1.0;
            hardware.intake.setPower(1.0);
        }
    }

    public void toggleRaiseLift() {
        if(hardware.liftVerticalSpeed == 0.5) {
            hardware.liftVerticalSpeed = 0.0;
            hardware.liftVertical.setPower(0.0);
        } else {
            hardware.liftVerticalSpeed = 0.5;
            hardware.liftVertical.setPower(0.5);
        }
    }

    public void toggleTranslateLift() {
        if(hardware.liftHorizontalSpeed == -0.4) {
            hardware.liftHorizontalSpeed = 0.4;
            hardware.liftHorizontal.setPower(0.4);
        } else {
            hardware.liftHorizontalSpeed = -0.4;
            hardware.liftHorizontal.setPower(-0.4);
        }
    }

    public void toggleTranslateLift(double power) {
        hardware.liftHorizontalSpeed = power;
        hardware.liftHorizontal.setPower(power);
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
}
