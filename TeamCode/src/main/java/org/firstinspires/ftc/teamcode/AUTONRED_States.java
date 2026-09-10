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

 package org.firstinspires.ftc.teamcode;

 import com.qualcomm.hardware.bosch.BNO055IMU;
 import com.qualcomm.robotcore.hardware.CRServo;
 import com.qualcomm.robotcore.hardware.Servo;
 import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
 import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
 import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
 import com.qualcomm.robotcore.hardware.DcMotor;
 import com.qualcomm.robotcore.util.ElapsedTime;
 
 import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
 import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
 import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
 import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
 
 import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
 import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
 import org.firstinspires.ftc.vision.VisionPortal;
 import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
 import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
 
 import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
 import org.firstinspires.ftc.vision.tfod.TfodProcessor;
 
 import java.util.List;
 
 /*
  * This OpMode illustrates the basics of TensorFlow Object Detection,
  * including Java Builder structures for specifying Vision parameters.
  *
  * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
  * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
  */
 @Autonomous(name = "RED AUTON STATES", group = "Concept")
 public class AUTONRED_States extends LinearOpMode {
     private boolean hasTargets;
     private int generatedScenario;
 
     DcMotor motorFrontLeft;
     DcMotor motorBackLeft;
     DcMotor motorFrontRight;
     DcMotor motorBackRight;
     CRServo autonServo;
     CRServo ramp;
     CRServo bucketArm;
     CRServo intake;
 
 
 
 
 
     BNO055IMU imu;
     Orientation lastAngles = new Orientation();
     double globalAngle;
 
     private ElapsedTime runtime = new ElapsedTime();
 
     static final double COUNTS_PER_MOTOR_REV = 538;
     static final double DRIVE_GEAR_REDUCTION = 1.75;
     static final double WHEEL_DIAMETER_INCHES = 3.78;
     static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
         (WHEEL_DIAMETER_INCHES * 3.14159);
     static final double DRIVE_SPEED = 0.6;
     static final double TURN_SPEED = 0.4;
 
     private static final boolean USE_WEBCAM = true; // true for webcam, false for phone camera
 
     // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
     // this is only used for Android Studio when using models in Assets.
     private static final String TFOD_MODEL_ASSET = "rednew.tflite"; // redTest1.tflite
     // TFOD_MODEL_FILE points to a model file stored onboard the Robot Controller's storage,
     // this is used when uploading models directly to the RC using the model upload interface.
     private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/redTest1.tflite";
     // Define the labels recognized in the model for TFOD (must be in training order!)
     private static final String[] LABELS = {
         "blue",
     };
 
 
     private double targetX;
 
     /**
      * The variable to store our instance of the TensorFlow Object Detection processor.
      */
     private TfodProcessor tfod;
 
     /**
      * The variable to store our instance of the vision portal.
      */
     private VisionPortal visionPortal;
     double MIN_POSITION = 0, MAX_POSITION = 1;
     @Override
     public void runOpMode() {
 
         // Initialize the IMU and its parameters.
         BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
         parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
         parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
         parameters.calibrationDataFile = "BNO055IMUCalibration.json";
         parameters.loggingEnabled = true;
         parameters.loggingTag = "IMU";
         parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
 
         imu = hardwareMap.get(BNO055IMU.class, "imu");
         imu.initialize(parameters);
 
         motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
         motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
         motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
         motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
         autonServo = hardwareMap.crservo.get("autonDropServo");
         ramp = hardwareMap.crservo.get("ramp");
         bucketArm = hardwareMap.crservo.get("bucketArm");
         intake = hardwareMap.crservo.get("intake");
 
         motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
         motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
         // motorBackRight.setDirection(DcMotor.Direction.REVERSE); // On expansion hub,
         // also needs to be reversed
 
         motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
         motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
         motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
         motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
 
         // The IMU does not initialize instantly. This makes it so the driver can see
         // when they can push Play without errors.
         telemetry.addData("Mode", "calibrating...");
         telemetry.update();
         while (!isStopRequested() && !imu.isGyroCalibrated()) {
             sleep(50);
             idle();
         }
 
         telemetry.addData("Status", "Resetting Encoders");
         telemetry.update();
 
         motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
 
         motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
 
         telemetry.addData("Path0", "Starting at %7d :%7d",
             motorBackLeft.getCurrentPosition(),
             motorBackRight.getCurrentPosition(),
             motorFrontLeft.getCurrentPosition(),
             motorFrontRight.getCurrentPosition());
         telemetry.update();
 
         // Tells the driver it is ok to start.
         telemetry.addData("Mode", "waiting for start");
         telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
         telemetry.update();
 
         initTfod();
 
         // Wait for the DS start button to be touched.
         telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
         telemetry.addData(">", "Touch Play to start OpMode");
         telemetry.update();
 
         waitForStart();
 
         //! Main Code
         //INSERT ENCODER DRIVE METHODS HERE!!!
         //frontLeft, frontRight, backLeft, backRight
         sleep(1000);
         telemetryTfod();
 
 
         encoderDrive(DRIVE_SPEED, 3, 3, 3, 3, 0.5);
 
 
         if (hasTargets) {
             // MIDDLE TARGET

             //! This is working, do not touch
 
             encoderDrive(0.4, 34, 34, 34, 34, 5); // Forward
             rotateRamp();
             encoderDrive(DRIVE_SPEED, 2, 2, 2, 2, 5); // Forward
             encoderDrive(DRIVE_SPEED, 24, -24, -24, 24, 5); // Right
             encoderDrive(DRIVE_SPEED, -18, -18, -18, -18, 5); // back
 
             rotate(-84, TURN_SPEED);
 
             encoderDrive(DRIVE_SPEED, 14, 14, 14, 14, 5); // Forward
             encoderDrive(DRIVE_SPEED, 1, -1, -1, 1, 5); // Move right 


             turnServo(1);
 
            sleep(1000);

             encoderDrive((DRIVE_SPEED * 0.5), -3, -3, -3, -3, 5); // Back Up
 
             encoderDrive(DRIVE_SPEED, 20, -20, -20, 20, 5); // Move right and park
             turnServo(-1); // Rotates "Jesus" back to original position
 
         } else {
 
             encoderDrive(DRIVE_SPEED, 10, -10, -10, 10, 2); // Strafe right to check
 
             sleep(1000);
             telemetryTfod();
 
             if (hasTargets) {
 
                 //TO THE RIGHT (PROP)
 
                 encoderDrive(DRIVE_SPEED, 22, 22, 22, 22, 4);
                 sleep(600);
                 rotate(-80, TURN_SPEED); // Rotate 80 degrees towards board (camera)
                 sleep(600);
                 encoderDrive(DRIVE_SPEED, 8, 8, 8, 8, 5); // Forwads before rotate
                 rotateRamp();
                 encoderDrive(DRIVE_SPEED, 16, 16, 16, 16, 5); // Robot is positioned at board
                 sleep(500);
                 encoderDrive(DRIVE_SPEED, 10.5, -10.5, -10.5, 10.5, 5); // Strafe Right
                 encoderDrive(DRIVE_SPEED, 2, 2, 2, 2, 2); // Forward
                 turnServo(1); // Rotates "Jesus" up to drop yellow pixel on backdrop
 
                 sleep(1000);
 
                 encoderDrive((DRIVE_SPEED * 0.5), -3, -3, -3, -3, 5); // Back Up
                 encoderDrive(DRIVE_SPEED, 12, -12, -12, 12, 12); // Park (Right Corner)
                 turnServo(-1); // Rotates "Jesus" back to original position
             } else {
                 //TO THE LEFT
 
                 rotate(-78, TURN_SPEED); // Turn camera facing board
                 sleep(500);
                 encoderDrive(DRIVE_SPEED, -21, 21, 21, -21, 3); // Strafe to left before backing
                 sleep(500);
                 encoderDrive(DRIVE_SPEED, -11, -11, -11, -11, 2); // Back Up
                 rotateRamp(); // Arm up and ramp moves 
                 encoderDrive(DRIVE_SPEED, 31, 31, 31, 31, 3); // Move Forward
                 sleep(500);
                 encoderDrive(DRIVE_SPEED, -6, 6, 6, -6, 2); // Strafe to left before rotating "Jesus"
                 encoderDrive(DRIVE_SPEED, 7.5, 7.5, 7.5, 7.5, 2); // Move Forward
                 turnServo(1); // Rotates "Jesus" up to drop yellow pixel on backdrop
 
                 sleep(500);

                 encoderDrive((DRIVE_SPEED * 0.5), -3, -3, -3, -3, 2); // Back Up
                 encoderDrive(DRIVE_SPEED, 30, -30, -30, 30, 5); // Park (Right Corner)
                 turnServo(-1); // Rotates "Jesus" back to original position

             }
         }
 
 
 
         if (opModeIsActive()) {
             while (opModeIsActive()) {
 
                 telemetryTfod();
 
                 // Push telemetry to the Driver Station.
                 telemetry.update();
 
                 // Share the CPU.
                 sleep(20);
             }
         }
 
         // Save more CPU resources when camera is no longer needed.
         visionPortal.close();
 
     } // end runOpMode()
 
     /**
      * Initialize the TensorFlow Object Detection processor.
      */
     private void initTfod() {
 
         // Create the TensorFlow processor by using a builder.
         tfod = new TfodProcessor.Builder()
 
 
 
             //.setModelAssetName(TFOD_MODEL_ASSET)
             .setModelFileName(TFOD_MODEL_FILE)
 
             //.setModelLabels(LABELS)
             //.setIsModelTensorFlow2(true)
             //.setIsModelQuantized(true)
             //.setModelInputSize(300)
             .setModelAspectRatio(16.0 / 9.0)
 
             .build();
 
         // Create the vision portal by using a builder.
         VisionPortal.Builder builder = new VisionPortal.Builder();
 
         // Set the camera (webcam vs. built-in RC phone camera).
         if (USE_WEBCAM) {
             builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
         } else {
             builder.setCamera(BuiltinCameraDirection.BACK);
         }
 
         // Choose a camera resolution. Not all cameras support all resolutions.
         //builder.setCameraResolution(new Size(640, 480));
 
         // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
         //builder.enableLiveView(true);
 
         // Set the stream format; MJPEG uses less bandwidth than default YUY2.
         //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);
 
         // Choose whether or not LiveView stops if no processors are enabled.
         // If set "true", monitor shows solid orange screen if no processors enabled.
         // If set "false", monitor shows camera view without annotations.
         //builder.setAutoStopLiveView(false);
 
         // Set and enable the processor.
         builder.addProcessor(tfod);
 
         // Build the Vision Portal, using the above settings.
         visionPortal = builder.build();
 
         // Set confidence threshold for TFOD recognitions, at any time.
         tfod.setMinResultConfidence(0.90f);
 
         // Disable or re-enable the TFOD processor at any time.
         //visionPortal.setProcessorEnabled(tfod, true);
 
     } // end method initTfod()
 
     /**
      * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
      */
     private void telemetryTfod() {
         List < Recognition > currentRecognitions = tfod.getRecognitions();
         telemetry.addData("# Objects Detected", currentRecognitions.size());
 
         if (currentRecognitions.size() > 0) {
             hasTargets = true;
         } else {
             hasTargets = false;
         }
 
         // Step through the list of recognitions and display info for each one.
         for (Recognition recognition: currentRecognitions) {
             targetX = (recognition.getLeft() + recognition.getRight()) / 2;
             double y = (recognition.getTop() + recognition.getBottom()) / 2;
 
             telemetry.addData("", " ");
             telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
             telemetry.addData("- Position", "%.0f / %.0f", targetX, y);
             telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
         } // end for() loop
 
     } // end method telemetryTfod()
 
     // Method for driving with encoder
     public void encoderDrive(double speed, double leftInches, double rightInches, double backleftInches, double backrightInches, double timeoutS) {
         //leftInches *= -1;
         //rightInches *= -1;
         //backleftInches *= -1;
         //backrightInches *= -1;
         int newLeftTarget;
         int newRightTarget;
         int newBackLeftTarget;
         int newBackRightTarget;
 
         // Ensure that the opmode is still active
         if (opModeIsActive()) {
 
             // Determine new target position, and pass to motor controller
             newLeftTarget = motorFrontLeft.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
             newRightTarget = motorFrontRight.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
             newBackLeftTarget = motorBackLeft.getCurrentPosition() + (int)(backleftInches * COUNTS_PER_INCH);
             newBackRightTarget = motorBackRight.getCurrentPosition() + (int)(backrightInches * COUNTS_PER_INCH);
             motorFrontLeft.setTargetPosition(newLeftTarget);
             motorFrontRight.setTargetPosition(newRightTarget);
             motorBackLeft.setTargetPosition(newBackLeftTarget);
             motorBackRight.setTargetPosition(newBackRightTarget);
 
             // Turn On RUN_TO_POSITION
             motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
             motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
             motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
             motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
 
             // reset the timeout time and start motion.
             runtime.reset();
             motorBackLeft.setPower(Math.abs(speed));
             motorBackRight.setPower(Math.abs(speed));
             motorFrontLeft.setPower(Math.abs(speed));
             motorFrontRight.setPower(Math.abs(speed));
 
             while (opModeIsActive() &&
                 (runtime.seconds() < timeoutS) &&
                 (motorBackLeft.isBusy() && motorBackRight.isBusy())) {
 
                 // Display it for the driver.
                 telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget, newBackLeftTarget,
                     newBackRightTarget);
                 telemetry.addData("Path2", "Running at %7d :%7d",
                     motorFrontLeft.getCurrentPosition(),
                     motorFrontRight.getCurrentPosition(),
                     motorBackLeft.getCurrentPosition(),
                     motorBackRight.getCurrentPosition());
                 telemetry.update();
             }
 
             // Stop all motion;
             motorBackLeft.setPower(0);
             motorBackRight.setPower(0);
             motorFrontLeft.setPower(0);
             motorFrontRight.setPower(0);
 
             // Turn off RUN_TO_POSITION
             motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
             motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
             motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
             motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
 
         }
 
     }
 
     // This method reads the IMU getting the angle. It automatically adjusts the
     // angle so that it is between -180 and +180.
     public double getAngle() {
         Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
 
         double deltaAngle = angles.firstAngle - lastAngles.firstAngle;
 
         if (deltaAngle < -180)
             deltaAngle += 360;
         else if (deltaAngle > 180)
             deltaAngle -= 360;
 
         globalAngle += deltaAngle;
 
         lastAngles = angles;
 
         return globalAngle;
     }
 
     // The method turns the robot by a specific angle, -180 to +180.
     public void rotate(int degrees, double power) {
         double backLeftPower, backRightPower, frontLeftPower, frontRightPower;
 
         resetAngle();
 
         // if the degrees are less than 0, the robot will turn right
         if (degrees < 0) {
             motorBackLeft.setPower(power);
             motorFrontLeft.setPower(power);
             motorBackRight.setPower(-power);
             motorFrontRight.setPower(-power);
         } else if (degrees > 0) // if greater than 0, turn left
         {
             motorBackLeft.setPower(-power);
             motorFrontLeft.setPower(-power);
             motorBackRight.setPower(power);
             motorFrontRight.setPower(power);
         } else
             return;
 
         // Repeatedly check the IMU until the getAngle() function returns the value
         // specified.
         if (degrees < 0) {
             while (opModeIsActive() && getAngle() == 0) {}
 
             while (opModeIsActive() && getAngle() > degrees) {}
         } else
             while (opModeIsActive() && getAngle() < degrees) {}
 
         // stop the motors after the angle has been found.
 
         motorBackLeft.setPower(0);
         motorBackRight.setPower(0);
         motorFrontLeft.setPower(0);
         motorFrontRight.setPower(0);
 
         // sleep for a bit to make sure the robot doesn't over shoot
 
         resetAngle();
     }
 
     // this method resets the angle so that the robot's heading is now 0
     public void resetAngle() {
         lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
 
         globalAngle = 0;
     }
 
     //public void servo to a postion
 
     public void turnServo(double servoPosition) {
         //autonServo.setPosition(Range.clip(servoPosition, MIN_POSITION, MAX_POSITION));
         if (servoPosition == 1) {
             autonServo.setPower(-0.2);
             sleep(2500);
             autonServo.setPower(0);
         } else {
             autonServo.setPower(1);
             sleep(3000);
             autonServo.setPower(0);
         }
 
     }
 
     public void rotateRamp() {
         // Move arm up
         bucketArm.setPower(1);
         sleep(1000);
         bucketArm.setPower(0);
 
         // Move ramp
         ramp.setPower(-1);
         intake.setPower(1);
         sleep(2000);
         ramp.setPower(0);
         intake.setPower(0);
 
 
 
         //Move arm down
         /*
         bucketArm.setPower(-0.5);
         sleep(1000);
         bucketArm.setPower(0);
         */
     }
 
     //
 
 
 } // End of the AUTONRED_States class