package org.firstinspires.ftc.teamcode;


public class RobotMovement {
    RobotHardware hardware;

    RobotMovement(Robot robot) {
        this.hardware = robot.hardware;
    }

    public void strafe(double x, double y, double h) {
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(h), 1);
        double frontLeftPower = (x  + y + h) / denominator;
        double backLeftPower = (x - y - h) / denominator;
        double frontRightPower = (x + y - h) / denominator;
        double backRightPower = (x - y + h) / denominator;

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
        if(hardware.liftSpeed == 0.5) {
            hardware.liftSpeed = 0.0;
            hardware.lift.setPower(0.0);
        } else {
            hardware.liftSpeed = 0.5;
            hardware.lift.setPower(0.5);
        }
    }

    public void toggleLowerLift() {
        if(hardware.liftSpeed == -0.2) {
            hardware.liftSpeed = 0.0;
            hardware.lift.setPower(0.0);
        } else {
            hardware.liftSpeed = -0.2;
            hardware.lift.setPower(-0.2);
        }
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
        } else {
            hardware.armPosition = 0.98;
            hardware.arm.setPosition(0.98);
        }
    }

    public void toggleArm(double position) {
        hardware.arm.setPosition(position);
    }
}
