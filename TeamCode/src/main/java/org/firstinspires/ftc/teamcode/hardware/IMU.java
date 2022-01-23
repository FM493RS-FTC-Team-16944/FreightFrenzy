package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.TelemLog;
import org.firstinspires.ftc.teamcode.models.Tuple;

public class IMU {
    private final BNO055IMU imu;
    private final TelemLog telemetry;
    private Orientation lastAngles;

    private double globalAngle;

    public IMU(String name, HardwareMap hardwareMap, TelemLog telemetry) {
        this.imu = hardwareMap.get(BNO055IMU.class, name);
        this.telemetry = telemetry;

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        imu.initialize(parameters);
        resetAngle();
    }

    private void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

        globalAngle = 0;
    }

    public Tuple<Double, Double> getAngle() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (globalAngle < -Math.PI)
            this.globalAngle += 2 * Math.PI;
        else if (globalAngle > Math.PI)
            this.globalAngle -= 2 * Math.PI;

        this.globalAngle += deltaAngle;
        lastAngles = angles;

        return new Tuple<>(this.globalAngle, deltaAngle);
    }

    public void calibrate(LinearOpMode teleOP) {
        while (!teleOP.isStopRequested() && !imu.isGyroCalibrated()) {
            teleOP.sleep(50);
            teleOP.idle();
        }
    }

    public void updateIMUReadings() {
        Tuple<Double, Double> angles = this.getAngle();

        String angleGlobal = RobotHardware.DECIMAL_FORMAT.format(angles.x);
        String deltaAngle = RobotHardware.DECIMAL_FORMAT.format(angles.y);

        this.telemetry.addData("Global Angle Encoder : " , angleGlobal);
        this.telemetry.addData("Delta Angle : " , deltaAngle);
    }
}
