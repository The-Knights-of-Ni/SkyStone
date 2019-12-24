package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

@Autonomous(name = "AutoEncoder", group = "Concept")

public class AutoEncoder extends LinearOpMode {
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "AR194VT/////AAABmV+h6Y9UMUSyo6qzOXtt2dAwNZ9bJ8m/6RSx/0vIwbrT4cjrgvkWuEXawFdMC7y6rbDpSZTcSs+eRZxKp0k43J6jKJktNwCLMF2iPA6yfQ6pNOIwOCoYIGC+uGdSi9+E+g9l7OH+zUWl6CXHyhUwbwTIFlduAIVaX0I2kpPuxJO4drMmZzEwsr7nHME98s/eNV30jACsP6yhUN/7w+CNEDcIvGM+J+16B978QXaGHa23ACXSkv0gXwLaztGPuPrLAfSd0kmnIaAgbDm0BUdTayFhVFaVU/VgvAjgZ7eT40BoOkAtvayDx+uPmjfTibskPk0n/eosVD7I2uxaBLHJ20w6xgOqCYlnWZ11axpyiECJ";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;


    @Override
    public void runOpMode() {
        // Create and set localizer parameters
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey =
                "AR194VT/////AAABmV+h6Y9UMUSyo6qzOXtt2dAwNZ9bJ8m/6RSx/0vIwbrT4cjrgvkWuEXawFdMC7y6rbDpSZTcSs+eRZxKp0k43J6jKJktNwCLMF2iPA6yfQ6pNOIwOCoYIGC+uGdSi9+E+g9l7OH+zUWl6CXHyhUwbwTIFlduAIVaX0I2kpPuxJO4drMmZzEwsr7nHME98s/eNV30jACsP6yhUN/7w+CNEDcIvGM+J+16B978QXaGHa23ACXSkv0gXwLaztGPuPrLAfSd0kmnIaAgbDm0BUdTayFhVFaVU/VgvAjgZ7eT40BoOkAtvayDx+uPmjfTibskPk0n/eosVD7I2uxaBLHJ20w6xgOqCYlnWZ11axpyiECJ";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        // Create localizer
        VuforiaLocalizer localizer = ClassFactory.createVuforiaLocalizer(parameters);

        // Set up the stone target listener
        VuforiaTrackables skystone = localizer.loadTrackablesFromAsset("Skystone");
        VuforiaTrackable stoneTarget = skystone.get(0);
        stoneTarget.setName("Stone Target");
        VuforiaTrackableDefaultListener stoneListener =
                (VuforiaTrackableDefaultListener)stoneTarget.getListener();

        // Active the listeners
        skystone.activate();

        DcMotor driveR = hardwareMap.dcMotor.get("fr");
        driveR.setDirection(DcMotorSimple.Direction.REVERSE);
        driveR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        DcMotor driveL = hardwareMap.dcMotor.get("fl");
        driveL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Wait for the robot driver to start the autonomous mode
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        waitForStart();

        driveL.setTargetPosition(-5000);

        driveL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        driveL.setPower(0.2);
        driveR.setPower(0.2);

        while (opModeIsActive() && stoneListener.getRawPose() == null) {
            idle();
        }

        driveL.setPower(0);
        driveR.setPower(0);

        // analyze skystone

        VectorF angles = anglesFromTarget (stoneListener);
        VectorF trans = navOffWall(stoneListener.getPose().getTranslation(), Math.toDegrees(angles.get(0)) - 90, new VectorF(500, 0, 0));

        if(trans.get(0) >0){
            driveL.setPower(0.02);
            driveR.setPower(-0.02);
        } else {
            driveL.setPower(-0.02);
            driveR.setPower(0.02);
        }

        do{
            if(stoneListener.getPose() != null) {
                trans = navOffWall(stoneListener.getPose().getTranslation(), Math.toDegrees(angles.get(0)) - 90, new VectorF(500, 0, 0));
            }
            idle();
        } while (opModeIsActive() && Math.abs(trans.get(0)) > 30);

        driveL.setPower(0);
        driveR.setPower(0);
    }


    public VectorF navOffWall(VectorF trans, double robotAngle, VectorF offWall){
        return new VectorF((float) (trans.get(0) - offWall.get(0) * Math.sin(Math.toRadians(robotAngle)) - offWall.get(2) * Math.cos(Math.toRadians(robotAngle))), trans.get(1), (float) (trans.get(2) + offWall.get(0) * Math.cos(Math.toRadians(robotAngle)) - offWall.get(2) * Math.sin(Math.toRadians(robotAngle))));
    }

    public VectorF anglesFromTarget(VuforiaTrackableDefaultListener image){
        float [] data = image.getRawPose().getData();
        float [] [] rotation = {{data[0], data[1]}, {data[4], data[5], data[6]}, {data[8], data[9], data[10]}};

        double thetaX = Math.atan2(rotation[2][1], rotation[2][2]);
        double thetaY = Math.atan2(-rotation[2][0], Math.sqrt(rotation[2][1] * rotation[2][1] + rotation[2][2] * rotation[2][2]));
        double thetaZ = Math.atan2(rotation[1][0], rotation[0][0]);
        return new VectorF((float)thetaX, (float)thetaY, (float)thetaZ);
    }
}