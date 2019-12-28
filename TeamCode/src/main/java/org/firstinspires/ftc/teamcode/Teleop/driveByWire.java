package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Robot;


@TeleOp(name="Drive by Wire", group="Exercises")
public class driveByWire extends LinearOpMode {
        private Robot robot;
        private BNO055IMU imu;
        double robotAngle;
        Orientation lastAngles = new Orientation();
        double                  globalAngle, power = .30;

        private void initOpMode() {
            //Initialize DC motor objects
            ElapsedTime timer = new ElapsedTime();
            robot = new Robot(this, timer);

        }

        // called when init button is  pressed.
        @Override
        public void runOpMode() throws InterruptedException
        {
            initOpMode();
            robot.frontLeftDriveMotor.setDirection(DcMotor.Direction.REVERSE);
            robot.rearLeftDriveMotor.setDirection(DcMotor.Direction.REVERSE);


            robot.frontLeftDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.frontRightDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.rearLeftDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.rearRightDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

            parameters.mode                = BNO055IMU.SensorMode.IMU;
            parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.loggingEnabled      = false;

            // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
            // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
            // and named "imu".
            imu = hardwareMap.get(BNO055IMU.class, "imu");

            imu.initialize(parameters);

            telemetry.addData("Mode", "calibrating...");
            telemetry.update();

            // make sure the imu gyro is calibrated before continuing.
            while (!isStopRequested() && !imu.isGyroCalibrated())
            {
                sleep(50);
                idle();
            }

            telemetry.addData("Mode", "waiting for start");
            telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
            telemetry.update();

            // wait for start button.

            waitForStart();

            telemetry.addData("Mode", "running");
            telemetry.update();

            sleep(1000);

            // drive until end of period.

            while (opModeIsActive())
            {
                //Get gamepad inputs
                double leftStickX = gamepad1.left_stick_x;
                double leftStickY = -gamepad1.left_stick_y;
                double rightStickX = gamepad1.right_stick_x;
                boolean aButton = gamepad1.a;
                boolean bButton = gamepad1.b;
                boolean dPadUp = gamepad1.dpad_up;
                boolean dPadDown = gamepad1.dpad_down;
                boolean dPadLeft = gamepad1.dpad_left;
                boolean dPadRight = gamepad1.dpad_right;

                double leftStickX2 = gamepad2.left_stick_x;
                double leftStickY2 = -gamepad2.left_stick_y;
                double rightStickX2 = gamepad2.right_stick_x;
                double rightStickY2 = gamepad2.right_stick_y;
                boolean aButton2 = gamepad2.a;
                boolean bButton2 = gamepad2.b;
                boolean dPadUp2 = gamepad2.dpad_up;
                boolean dPadDown2 = gamepad2.dpad_down;
                boolean dPadLeft2 = gamepad2.dpad_left;
                boolean dPadRight2 = gamepad2.dpad_right;
                boolean bumperLeft2 = gamepad2.left_bumper;
                boolean bumperRight2 = gamepad2.right_bumper;

                robotAngle = imu.getAngularOrientation().firstAngle;

                double r = 0.3;
                double goalAngle = 0;
                double correctionAmount = robotAngle - goalAngle;
                //double correctedAngle = goalAngle - correctionAmount;
                if(Math.abs(correctionAmount) >= 2) {
                    double lrPower = r;
                    double lfPower = r;
                    double rrPower = r;
                    double rfPower = r;
                    robot.rearLeftDriveMotor.setPower(lrPower);
                    robot.frontLeftDriveMotor.setPower(lfPower);
                    robot.rearRightDriveMotor.setPower(rrPower);
                    robot.frontRightDriveMotor.setPower(rfPower);
                } else if (Math.abs(correctionAmount) <= 10) {
                    robot.rearLeftDriveMotor.setPower(0.1);
                    robot.frontLeftDriveMotor.setPower(0.1);
                    robot.rearRightDriveMotor.setPower(0.1);
                    robot.frontRightDriveMotor.setPower(0.1);
                } else {
                    robot.rearLeftDriveMotor.setPower(0);
                    robot.frontLeftDriveMotor.setPower(0);
                    robot.rearRightDriveMotor.setPower(0);
                    robot.frontRightDriveMotor.setPower(0);
                }


                // Use gyro to drive in a straight line.
                telemetry.addData("Correction Angle: ", correctionAmount );
                telemetry.addData("Robot Angle: ", robotAngle );
                telemetry.addData("1 imu heading", lastAngles.firstAngle);
                telemetry.addData("2 global heading", globalAngle);
                //telemetry.addData("3 correction", correction);
                telemetry.update();

//                robot.drive.frontLeft.setPower(power);
//                robot.drive.rearLeft.setPower(power);
//                robot.drive.frontRight.setPower(power);
//                robot.drive.rearRight.setPower(power);
                resetAngle();
            }



            // turn the motors off.
            robot.frontLeftDriveMotor.setPower(0);
            robot.frontRightDriveMotor.setPower(0);
            robot.rearLeftDriveMotor.setPower(0);
            robot.rearRightDriveMotor.setPower(0);
        }

        /**
         * Resets the cumulative angle tracking to zero.
         */
        private void resetAngle()
        {
            lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            globalAngle = 0;
        }
}
