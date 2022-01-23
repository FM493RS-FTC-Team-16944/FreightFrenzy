package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.movement.Odometry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.TelemLog;
import org.firstinspires.ftc.teamcode.models.Mode;

public class RobotHardware {
    public final Manipulator manipulator;
    public final MecanumDriveTrain driveTrain;
    public final IMU imu;
    public final State state;
    public final Odometry odometry;
    public final TelemLog telemetry;

    public HardwareMap hardwareMap;
    public Mode currentMode = Mode.DRIVER_CONTROL;

    public class State {
        public double intakeSpeed = 0.0;
        public double flyWheelSpeed = 0.0;
        public double liftSpeed = 0.0;
        public double clawPosition = 0.0;
        public double armPosition = 0.0;
        public boolean freightLoaded = true;
        public boolean armOpen = false;
    }


    public RobotHardware(Robot robot) {
        hardwareMap = robot.teleOP.hardwareMap;

        this.state = new State();
        this.telemetry = robot.telemLog;

        this.driveTrain = new MecanumDriveTrain(
                "FrontLeft",
                "BackLeft",
                "FrontRight",
                "BackRight",
                "BackLeft",
                "Flywheel",
                "BackLeft",
                hardwareMap,
                this.telemetry
        );

        this.manipulator = new Manipulator(
                hardwareMap,
                this.telemetry
        );

        this.imu = new IMU(
                "imu",
                hardwareMap,
                this.telemetry
        );

        this.imu.calibrate(robot.teleOP);

        this.odometry = new Odometry(this);
    }

    public void outputReadings() {
        this.odometry.updateOdometryReadings();
        this.manipulator.updatePositionReadings();
        this.driveTrain.updateEncoderReadings();
        this.imu.updateIMUReadings();

        this.telemetry.update();
    }
}