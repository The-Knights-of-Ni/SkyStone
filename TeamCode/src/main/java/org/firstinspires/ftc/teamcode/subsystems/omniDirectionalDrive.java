package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


public class omniDirectionalDrive {

    //Sensors
    private BNO055IMU imu;

    public omniDirectionalDrive(OpMode opMode, ElapsedTime timer){
        hardwareMap = opMode.hardwareMap;
        this.timer = timer;
        init();
    }
    public omniDirectionalDrive (){
        init();
    }
    public void init(){


        //Sensors
        imu = hardwareMap.get(BNO055IMU.class, "imu");
//        colorSensor = hardwareMap.colorSensor.get("color");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();


        imu.initialize(parameters);
        position = imu.getPosition();

        //Subsystems
        drive = new Drive(frontLeftDriveMotor, frontRightDriveMotor, rearLeftDriveMotor, rearRightDriveMotor, imu, timer);

    }
}
