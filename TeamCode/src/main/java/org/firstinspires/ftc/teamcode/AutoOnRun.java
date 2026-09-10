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

package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;


import java.util.List;


/**
 * This 2022-2023 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine which image is being presented to the robot.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Autonomous(name = "Auto On Run")

public class AutoOnRun extends LinearOpMode {
    private DcMotor motorFrontLeft = null; //Declares stuff ON ROBOT
    private DcMotor motorBackLeft = null;
    private DcMotor motorFrontRight = null;
    private DcMotor motorBackRight = null;

    /* Declare OpMode members. */
    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 840 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    /*
     * Specify the source for the Tensor Flow Model.
     * If the TensorFlowLite object model is included in the Robot Controller App as an "asset",
     * the OpMode must to load it using loadModelFromAsset().  However, if a team generated model
     * has been downloaded to the Robot Controller's SD FLASH memory, it must to be loaded using loadModelFromFile()
     * Here we assume it's an Asset.    Also see method initTfod() below .
     */
    private static final String TFOD_MODEL_ASSET = "ronaldo.tflite";
    // private static final String TFOD_MODEL_FILE  = "/sdcard/FIRST/tflitemodels/CustomTeamModel.tflite";


    private static final String[] LABELS = {
            "0 Red",
            "1 Green",
            "2 Blue"
    };

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
            "AcJWPiP/////AAABmRv0x6fBjk9rr2cVNe/65PMBQNZEE0OHUmJE6gJQfJrlvCAt/5rKCJ/7Sz9zJxCm+GXOmSArVpbJfrEOkQF9nRfrq3rkh8xHnSKc4tIl2KT7x9s2ev8d8S/mzJ+1NjqV7CBPVS7dFUqzcTgoqnuZgUJG/pzAFgCBJdkQdUDUMy8/qOdWuz8B8GthAKc5cmSFyBvvwk7y4Edmv/pqIwLiwP+M2H/13o/jySsQu6OctKGUSUMpvwX0Zd6BrmeaA9EVzAyWoURqzPkwQN1PBSUQVujQ6s1KZEDhhWs6EDzrcL66P35+7GVNSWEJYFTzdxeOjFNyv/tl5SoaKK0my47Fxid2Ta0Lm/OZx991/+kUBBga";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        // Claw Motors (Servo)
        Servo claw = hardwareMap.servo.get("reaprClaw");// name of server on control hub is reaprClaw
        // On port:
        double clawPosition = 0.0;
        final double clawSpeed = 0.05;// change to 100th when button is hold
        final double clawMinRange = 0.0;
        final double clawMaxRange = 0.55;

        clawPosition = Range.clip(clawPosition, clawMinRange, clawMaxRange);
        claw.setPosition(clawPosition);


        // Meccanum Drivetrain
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("motorBackRight");



        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        //motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);  // This was connected on the expansion hub, it needs to be reversed


        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                motorFrontLeft.getCurrentPosition(),
                motorFrontRight.getCurrentPosition(),
                motorBackLeft.getCurrentPosition(),
                motorBackRight.getCurrentPosition());
        telemetry.update();

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {

            // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
            // first.
            initVuforia();
            initTfod();

            /**
             * Activate TensorFlow Object Detection before we wait for the start command.
             * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
             **/
            if (tfod != null) {
                tfod.activate();

                // The TensorFlow software will scale the input images from the camera to a lower resolution.
                // This can result in lower detection accuracy at longer distances (> 55cm or 22").
                // If your target is at distance greater than 50 cm (20") you can increase the magnification value
                // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
                // should be set to the value of the images used to create the TensorFlow Object Detection model
                // (typically 16/9).
                tfod.setZoom(1.2, 16.0/9.0);
            }

            sleep(4000);

            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Objects Detected", updatedRecognitions.size());


                    // step through the list of recognitions and display image position/size information for each one
                    // Note: "Image number" refers to the randomized image orientation/number

                    int counter=0;
                    float maxConf = 0.0f;
                    int coneNumber =0;
                    for (Recognition recognition : updatedRecognitions) {
                        if(counter==0){
                            counter++;

                            double col = (recognition.getLeft() + recognition.getRight()) / 2 ;
                            double row = (recognition.getTop()  + recognition.getBottom()) / 2 ;
                            double width  = Math.abs(recognition.getRight() - recognition.getLeft()) ;
                            double height = Math.abs(recognition.getTop()  - recognition.getBottom()) ;

                            telemetry.addData(""," ");
                            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100 );
                            telemetry.addData("- Position (Row/Col)","%.0f / %.0f", row, col);
                            telemetry.addData("- Size (Width/Height)","%.0f / %.0f", width, height);
                            telemetry.update();


                            if(recognition.getConfidence()>maxConf){
                                maxConf = recognition.getConfidence();
                                if(recognition.getLabel()=="0 Red"){
                                    coneNumber =0;
                                }else if(recognition.getLabel()=="1 Green"){
                                    coneNumber =1;
                                }else{ // Blue
                                    coneNumber =2;

                                }
                            }

                        }

                    }

                    if(coneNumber==0){

                        encoderDrive(DRIVE_SPEED,  -16,  -16, -16, -16, 200.0);  // 200 second timeout, no need for it yet
                        encoderDrive(DRIVE_SPEED,  18,  -18, -18, 18, 200.0);  // 200 second timeout, no need for it yet
                        encoderDrive(DRIVE_SPEED,  -4,  -4, -4, -4, 200.0);  // 200 second timeout, no need for it yet


                    }else if(coneNumber==1){

                        encoderDrive(DRIVE_SPEED,  -22,  -22, -22, -22, 200.0);  // 200 second timeout, no need for it yet


                    }else{ // Blue

                        encoderDrive(DRIVE_SPEED,  -16,  -16, -16, -16, 200.0);  // 200 second timeout, no need for it yet
                        encoderDrive(DRIVE_SPEED,  -18,  18, 18, -18, 200.0);  // 200 second timeout, no need for it yet
                        encoderDrive(DRIVE_SPEED,  -4,  -4, -4, -4, 200.0);  // 200 second timeout, no need for it yet

                    }

                    telemetry.update();
                }else{
                    encoderDrive(DRIVE_SPEED,  -16,  -16, -16, -16, 200.0);  // 200 second timeout, no need for it yet
                    encoderDrive(DRIVE_SPEED,  -18,  18, 18, -18, 200.0);  // 200 second timeout, no need for it yet
                    encoderDrive(DRIVE_SPEED,  -4,  -4, -4, -4, 200.0);  // 200 second timeout, no need for it yet
                }
            }else{
                encoderDrive(DRIVE_SPEED,  -16,  -16, -16, -16, 200.0);  // 200 second timeout, no need for it yet
                encoderDrive(DRIVE_SPEED,  -18,  18, 18, -18, 200.0);  // 200 second timeout, no need for it yet
                encoderDrive(DRIVE_SPEED,  -4,  -4, -4, -4, 200.0);  // 200 second timeout, no need for it yet

            }

        }
    }

    public void encoderDrive(double speed, double leftInches, double rightInches, double leftBackInches, double rightBackInches, double timeoutS) {

        int newLeftTarget;
        int newRightTarget;
        int newLeftBackTarget;
        int newRightBackTarget;

        // Meccanum Drivetrain
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("motorBackRight");

        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        //motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);  // This was connected on the expansion hub, it needs to be reversed


        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = motorFrontLeft.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = motorFrontRight.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newLeftBackTarget = motorBackLeft.getCurrentPosition() + (int)(leftBackInches * COUNTS_PER_INCH);
            newRightBackTarget = motorBackRight.getCurrentPosition() + (int)(rightBackInches * COUNTS_PER_INCH);
            motorFrontLeft.setTargetPosition(newLeftTarget);
            motorFrontRight.setTargetPosition(newRightTarget);
            motorBackLeft.setTargetPosition(newLeftBackTarget);
            motorBackRight.setTargetPosition(newRightBackTarget);

            // Turn On RUN_TO_POSITION
            motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            motorFrontLeft.setPower(Math.abs(speed));
            motorFrontRight.setPower(Math.abs(speed));
            motorBackLeft.setPower(Math.abs(speed));
            motorBackRight.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (motorFrontLeft.isBusy() && motorFrontRight.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        motorFrontLeft.getCurrentPosition(),
                        motorFrontRight.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            motorFrontLeft.setPower(0);
            motorFrontRight.setPower(0);
            motorBackLeft.setPower(0);
            motorBackRight.setPower(0);

            // Turn off RUN_TO_POSITION
            motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.70f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
        tfod.loadModelFromFile(TFOD_MODEL_ASSET, LABELS);
        // tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
    }
}
