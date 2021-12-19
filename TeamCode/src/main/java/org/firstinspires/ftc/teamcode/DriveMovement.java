package org.firstinspires.ftc.teamcode;

public class DriveMovement {
    RobotHardware hardware;

    DriveMovement(Robot robot) {
        this.hardware = robot.hardware;
    }

    public void strafe() {

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
        if(hardware.liftHorizontalSpeed == 0.5) {
            hardware.liftHorizontalSpeed = 0.0;
            hardware.liftHorizontal.setPower(0.0);
        } else {
            hardware.liftHorizontalSpeed = 0.5;
            hardware.liftHorizontal.setPower(0.5);
        }
    }
}
