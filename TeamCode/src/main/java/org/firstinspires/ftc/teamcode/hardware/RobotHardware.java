package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Odometry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.models.Mode;

public class RobotHardware {
    public final Manipulator manipulator;
    public final MecanumDriveTrain driveTrain;
    public final IMU imu;
    public final State state;
    public final Odometry odometry;
    public final Telemetry telemetry;

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

        this.driveTrain = new MecanumDriveTrain(
                "FrontLeft",
                "BackLeft",
                "FrontRight",
                "BackRight",
                "BackLeft",
                "Flywheel",
                "BackLeft",
                hardwareMap,
                robot.teleOP.telemetry
        );

        this.manipulator = new Manipulator(
                hardwareMap,
                robot.teleOP.telemetry
        );

        this.imu = new IMU(
                "imu",
                robot.teleOP.hardwareMap,
                robot.teleOP.telemetry
        );

        this.imu.calibrate(robot.teleOP);

        this.odometry = new Odometry(this);

        this.telemetry = robot.teleOP.telemetry;
    }
}