package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;

/**
 * Created by tarunsingh on 9/24/17.
 */

@TeleOp(name = "TeleopMark1")
public class TeleopMark1 extends LinearOpMode {
    //Declare DC motor objects
    private Robot robot;

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

    int winchCurrentPosition = 0;
    int winchTargetPosition;
    double timePre;
    double timeCurrent;
    double leftStickY2Pre;
    double leftStickY2Current;
    double winchSpeed;
    double deltaT;
    double winchDeltaD;
    ElapsedTime timer;


    @Override
    public void runOpMode() throws InterruptedException {
        initOpMode();
        waitForStart();
        while(opModeIsActive()) {
            //Get gamepad inputs
            leftStickX = gamepad1.left_stick_x;
            leftStickY = -gamepad1.left_stick_y;
            rightStickX = gamepad1.right_stick_x;
            aButton = gamepad1.a;
            bButton = gamepad1.b;
            dPadUp = gamepad1.dpad_up;
            dPadDown = gamepad1.dpad_down;
            dPadLeft = gamepad1.dpad_left;
            dPadRight = gamepad1.dpad_right;

            leftStickX2 = gamepad2.left_stick_x;
            leftStickY2 = -gamepad2.left_stick_y;
            rightStickX2 = gamepad2.right_stick_x;
            rightStickY2 = gamepad2.right_stick_y;
            aButton2 = gamepad2.a;
            bButton2 = gamepad2.b;
            dPadUp2 = gamepad2.dpad_up;
            dPadDown2 = gamepad2.dpad_down;
            dPadLeft2 = gamepad2.dpad_left;
            dPadRight2 = gamepad2.dpad_right;
            bumperLeft2 = gamepad2.left_bumper;
            bumperRight2 = gamepad2.right_bumper;


            timeCurrent = timer.nanoseconds();

            double[] motorPowers = calcMotorPowers(leftStickX, leftStickY, rightStickX);
            robot.rearLeftDriveMotor.setPower(motorPowers[0]);
            robot.frontLeftDriveMotor.setPower(motorPowers[1]);
            robot.rearRightDriveMotor.setPower(motorPowers[2]);
            robot.frontRightDriveMotor.setPower(motorPowers[3]);

            robot.armTilt.setPower(Math.pow(rightStickY2, 1.0));

            //930mm, 8400 encoder count
            if((Math.abs(leftStickY2) < 0.5)){
                robot.xRailWinch.setPower(0.35);
            }
            else{
                robot.xRailWinch.setPower(0.7);
            }


            deltaT = timeCurrent - timePre;
            if(leftStickY2 >= 0.1){
                winchSpeed = (leftStickY2 - 0.1) * (robot.drive.getWinchMaxSpeed() / 0.9);
            }
            else if(leftStickY2  <= -0.1){
                winchSpeed = (leftStickY2 + 0.1) * (robot.drive.getWinchMaxSpeed() / 0.9);
            }
            else{
                winchSpeed = 0.0;
            }
            winchCurrentPosition = robot.xRailWinch.getCurrentPosition();
            winchTargetPosition = (int) (winchCurrentPosition + (winchSpeed * deltaT));
            if((winchTargetPosition <= 8410) && (winchTargetPosition >= 0)){
                robot.xRailWinch.setTargetPosition(winchTargetPosition);
            }


            telemetry.addData("Left Stick Y2", leftStickY2);
            telemetry.addData("Right Stick Y2", rightStickY2);
            telemetry.addData("Right Stick X", rightStickX);
            telemetry.addData("deltaT", deltaT);

//            telemetry.addData("", "");
//            telemetry.addData("Left Rear Power", robot.rearLeftDriveMotor.getPower());
//            telemetry.addData("Left Front Power", robot.frontLeftDriveMotor.getPower());
//            telemetry.addData("Right Rear Power", robot.rearRightDriveMotor.getPower());
//            telemetry.addData("Right Front Power", robot.frontRightDriveMotor.getPower());
            telemetry.update();
            timePre = timeCurrent;
        }
    }
    private void initOpMode() {
        //Initialize DC motor objects
        timer = new ElapsedTime();
        robot = new Robot(this, timer);
        robot.xRailWinch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.xRailWinch.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.xRailWinch.setTargetPosition(0);
        timeCurrent = timer.milliseconds();
        timePre = timer.nanoseconds();
//        robot.init();
//        lrDrive = hardwareMap.dcMotor.get("lrDrive");
//        lfDrive = hardwareMap.dcMotor.get("lfDrive");
//        rrDrive = hardwareMap.dcMotor.get("rrDrive");
//        rfDrive = hardwareMap.dcMotor.get("rfDrive");

        //Set directions
//        lrDrive.setDirection(DcMotor.Direction.REVERSE);
//        lfDrive.setDirection(DcMotor.Direction.REVERSE);
//        rrDrive.setDirection(DcMotor.Direction.FORWARD);
//        rfDrive.setDirection(DcMotor.Direction.FORWARD);
    }

    private double[] calcMotorPowers(double leftStickX, double leftStickY, double rightStickX) {
        double r = Math.hypot(leftStickX, leftStickY);
        double robotAngle = Math.atan2(leftStickY, leftStickX) - Math.PI / 4;
        double lrPower = r * Math.sin(robotAngle) + rightStickX;
        double lfPower = r * Math.cos(robotAngle) + rightStickX;
        double rrPower = r * Math.cos(robotAngle) - rightStickX;
        double rfPower = r * Math.sin(robotAngle) - rightStickX;
        return new double[]{lrPower, lfPower, rrPower, rfPower};
    }

    private double calcWinchPower(double leftStickY2, double maxPower){
        double power;
        if(leftStickY2 > maxPower){
            power = maxPower;
        }
        else if(leftStickY2 < -maxPower){
            power = -maxPower;
        }
        else
        {
            power = Math.round(leftStickY2 * 100.0) / 100.0;
        }
        return power;
    }
}