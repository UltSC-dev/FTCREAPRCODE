/* Base code from:
https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html

Main TeleOP (2 Driver)

Ports:
Motors:
- Spinner Motor - Control 0
- Front Left Wheel - Control 1
- Back Left Wheel - Control 2
- Muscle - Control 3
- Worm Gear - Expansion 0
- Front Right Wheel - Expansion 1
- Back Right Wheel - Expansion 2
- Spool - Expansion 3 (not using anymore)

Servos: 
- none - Control 0
- Intake - Control 1
- Ramp - Control 2
- Drone Launcher - Control 3
- Hinge - Control 4
- Auton Servo - Control 5
- Arm - Expansion 0
- none - Expansion 1
- none - Expansion 2
- Bucket - Expansion 3
- none - Expansion 4
- none - Expansion 5
- 
*/

package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "TeleOP")

public class Reapr_Main_TeleOP extends LinearOpMode {
    double hingePosition;
    double  MIN_POSITION = 0, MAX_POSITION = 1;
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration

        // Meccanum Drivetrain
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("motorBackRight");

        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        // motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);  // This was connected on the expansion hub, it needs to be reversed

        DcMotor wormGear = hardwareMap.dcMotor.get("wormGear"); // Port 0
        DcMotor muscle = hardwareMap.dcMotor.get("muscle"); // Port 3
        // DcMotor spinner = hardwareMap.dcMotor.get("spinnerMotor"); // Port 0 (intake is a servo now)
        CRServo intake = hardwareMap.crservo.get("intake");
        CRServo ramp = hardwareMap.crservo.get("ramp");
        CRServo autonServo = hardwareMap.crservo.get("autonDropServo");

        Servo hinge = hardwareMap.servo.get("hinge");

        //DcMotor spool = hardwareMap.dcMotor.get("spool"); // Port 3

        // Airplane launcher setup
        CRServo launcher = hardwareMap.crservo.get("launcher");
        CRServo bucketArm = hardwareMap.crservo.get("bucketArm");
        CRServo bucket = hardwareMap.crservo.get("bucket");


        /*
        double launcherPosition = 0.5;
        final double launcherSpeed = 0.1;// change to 100th when button is hold
        final double launcherMinRange = 0.3;
        final double launcherMaxRange = 0.55;
        */

        // Rack and pinion setup


        waitForStart();

        hingePosition = 0.1;

        if (isStopRequested()) return;

        boolean isSlowMode = false;
        double dividePower=1.0;

        int currentPosition = 0;

        boolean keepMoving = false;

        while (opModeIsActive()) {
            //! Control Speed
            if(isSlowMode){
                dividePower=1.5;
            }else{
                dividePower=1.0;
            }

            if(gamepad1.left_stick_button || gamepad1.dpad_right){
                if(isSlowMode){
                    isSlowMode=false;
                    sleep(500);
                }else{
                    isSlowMode=true;
                    sleep(500);
                }
            }

            //! Mecccanum controls
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;


            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator / dividePower; //Positive rotation results in forward & right motion
            double backLeftPower = (y - x + rx) / denominator / dividePower; //Positive rotation results in forward & left motion
            double frontRightPower = (y - x - rx) / denominator / dividePower; //Positive rotation results in forward & left motion
            double backRightPower = (y + x - rx) / denominator / dividePower; //Positive rotation results in forward & right motion

            motorFrontLeft.setPower(frontLeftPower);
            motorBackLeft.setPower(backLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(backRightPower);

            //! Misc motor controls
            // Worm Gear motor Controls
            // For hanging arm
            if (gamepad2.a){ // Move down
                wormGear.setPower(1);
                muscle.setPower(1);
            }else if (gamepad2.y){ // Move up
                wormGear.setPower(-1);
                muscle.setPower(-1);
            }else{
                wormGear.setPower(0);
                muscle.setPower(0);
            }



            // Spinner (intake) motor

            if(keepMoving && gamepad1.a){
                keepMoving = false;
                sleep(200);
            }else if(!keepMoving && gamepad1.a){
                keepMoving = true; 
                sleep(200);
            }
            
            
            if (keepMoving){ // Move up
                intake.setPower(1);
                ramp.setPower(-1);
            }else if (gamepad1.y){ // Move down
                intake.setPower(-1);
                ramp.setPower(1);
            }else{
                intake.setPower(0);
                ramp.setPower(0);
            }
            



            //! Servo Controls

            // Drone Launcher

            //telemetry.addData("Drone launcher", "%.2f", launcherPosition); //displays the values on the driver hub
            //telemetry.update();
            if (gamepad1.b) {
                launcher.setPower(-1);
                //telemetry.update();
            }
            else if (gamepad1.x){
                launcher.setPower(1);
                //telemetry.update();
            }


            // Bucket arm
            if (gamepad2.dpad_down) {
                bucketArm.setPower(-1);
                //telemetry.update();
            }else if (gamepad2.dpad_up){
                bucketArm.setPower(1);
            }else{
                bucketArm.setPower(0);
            }

            // Jesus
            if (gamepad1.dpad_up){
                autonServo.setPower(1);
            }else if (gamepad1.dpad_down){
                autonServo.setPower(-1);
            }else{
                autonServo.setPower(0);
            }

            // Bucket
            if(gamepad2.b){
                bucket.setPower(1);
            }else if(gamepad2.x){
                bucket.setPower(-1);
            }else{
                bucket.setPower(0);
            }

            // move hinge down on dpad left button if not already at lowest position.
            if (gamepad2.dpad_left && hingePosition > MIN_POSITION){
                hingePosition -= .01;
                hinge.setPosition(Range.clip(hingePosition, MIN_POSITION, MAX_POSITION));

            }

            // move hinge up on dpad right button if not already at the highest position.
            if (gamepad2.dpad_right && hingePosition < MAX_POSITION){
                hingePosition += .01;
                hinge.setPosition(Range.clip(hingePosition, MIN_POSITION, MAX_POSITION));

            } 
            
            telemetry.addData("hinge servo", "position=" + hingePosition + "  actual=" + hinge.getPosition());
            telemetry.update();

            // Spool
            /*
           if (gamepad2.dpad_left){
              spool.setPower(1);
            }
            spool.setPower(0);
            if (gamepad2.dpad_right) {
                spool.setPower(-1);
            } spool.setPower(0);
            */
        }
    }
}