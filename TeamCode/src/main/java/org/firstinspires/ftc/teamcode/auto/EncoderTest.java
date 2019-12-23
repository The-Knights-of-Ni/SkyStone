package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;

/**
 * Created by tarunsingh on 12/5/17.
 */

@Autonomous (name="Encoder Test")
public class EncoderTest extends LinearOpMode {
    private static final int targetPosition = 315;
    private static final double maxPower = 0;
    private Robot robot;

    public void initOpMode(){
        ElapsedTime timer = new ElapsedTime();
        robot = new Robot(this, timer);
        robot.xRailWinch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.xRailWinch.setTargetPosition(0);
        robot.xRailWinch.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void runOpMode() {
        initOpMode();
        waitForStart();
        robot.timer.reset();


        robot.drive.frontLeft.setTargetPosition(targetPosition);
        robot.drive.frontRight.setTargetPosition(targetPosition);
        robot.drive.rearLeft.setTargetPosition(targetPosition);
        robot.drive.rearRight.setTargetPosition(targetPosition);

        robot.drive.setDrivePower(maxPower);

//        robot.rearLeftDriveMotor.setPower(maxPower);
//        robot.frontLeftDriveMotor.setPower(maxPower);
//        robot.rearRightDriveMotor.setPower(maxPower);
//        robot.frontRightDriveMotor.setPower(maxPower);

//        while (opModeIsActive() && robot.drive.frontLeft.isBusy() && robot.drive.frontRight.isBusy()) {
////            telemetry.addData("Front Left", robot.drive.frontLeft.getCurrentPosition());
////            telemetry.addData("Front Right", robot.drive.frontRight.getCurrentPosition());
////            telemetry.addData("Rear Left", robot.drive.rearLeft.getCurrentPosition());
////            telemetry.addData("Rear Right", robot.drive.rearRight.getCurrentPosition());
//            telemetry.addData("Winch", robot.xRailWinch.getCurrentPosition());
//
//            telemetry.update();
//        }
//        sleep(1000);
        telemetry.addData("Winch", robot.xRailWinch.getCurrentPosition());

    }
}