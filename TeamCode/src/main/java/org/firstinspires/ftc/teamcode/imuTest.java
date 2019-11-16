package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by Elijah R. on 11/16/2019.
 */

public class imuTest extends LinearOpMode {
    public String name;
    private HardwareMap hardwareMap;

    //Sensors
    private BNO055IMU imu;

    //Subsystems
    public imuTest(OpMode opMode){
        hardwareMap = opMode.hardwareMap;
        init1();
    }

    public imuTest(){
        init1();
    }
    public void init1(){
        //Sensors
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();


        imu.initialize(parameters);

    }
    @Override public void runOpMode() {
        init1();
        while(opModeIsActive()) {
            telemetry.addData(String.format("Position: %d"), "0.03f", imu.getPosition());
            telemetry.update();
        }
    }
}


