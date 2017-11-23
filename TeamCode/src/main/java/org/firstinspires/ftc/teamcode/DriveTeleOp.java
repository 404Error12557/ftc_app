/* Copyright (c) 2017 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import static android.os.SystemClock.sleep;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Basic: Iterative OpMode", group="Iterative Opmode")
//@Disabled
public class DriveTeleOp extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private boolean speedFlag = false;

    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor motorRF = null;
    private DcMotor motorRB = null;
    private DcMotor motorLF = null;
    private DcMotor motorLB = null;
    private DcMotor motorLift = null;
    private DcMotor motorArm = null;
    private DcMotor motorLift1 = null;

    private CRServo liftServoRight= null;
    private CRServo liftServoLeft= null;
    private CRServo armServoRight= null;
    private CRServo armServoLeft= null;
    private Servo clawServo= null;
    private double linearPosition = 0.05;
    static final double INCREMENT   = 0.01;



    // amount to slew servo each CYCLE_MS cycle


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        //SS leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        //SS rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        liftServoRight = hardwareMap.get(CRServo.class, "lift_servo_right");
        liftServoLeft = hardwareMap.get(CRServo.class, "lift_servo_left");

        armServoRight = hardwareMap.get(CRServo.class, "arm_servo_right");
        armServoLeft = hardwareMap.get(CRServo.class, "arm_servo_left");
        clawServo = hardwareMap.get(Servo.class, "claw_servo");
        clawServo.scaleRange(0.4,0.8);





        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
      //SS  leftDrive.setDirection(DcMotor.Direction.REVERSE);
      //SS  rightDrive.setDirection(DcMotor.Direction.FORWARD);

        motorLF  = hardwareMap.get(DcMotor.class, "mLF");
        motorRF = hardwareMap.get(DcMotor.class, "mRF");
        motorLB = hardwareMap.get(DcMotor.class, "mLB");
        motorRB = hardwareMap.get(DcMotor.class, "mRB");
        motorLift = hardwareMap.get(DcMotor.class, "mLift");
        motorArm = hardwareMap.get(DcMotor.class, "mArm");
        motorLift1 =hardwareMap.get(DcMotor.class, "mLift1");

       motorLF.setDirection(DcMotor.Direction.REVERSE);
        //motorLB.setDirection(DcMotor.Direction.REVERSE);
        //motorRF.setDirection(DcMotor.Direction.FORWARD);
        motorRB.setDirection(DcMotor.Direction.REVERSE);
       // liftServoLeft.setDirection(CRServo.Direction.REVERSE);
        motorArm.setDirection(DcMotor.Direction.REVERSE);

        motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);//restsets the encoder
        motorArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);//runs to use the encoder


        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower;
        double rightPower;

        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
       // double drive = -gamepad1.right_stick_y;
        //double turn  =  gamepad1.right_stick_x;
       // double turn  =  gamepad1.right_stick_x;

       // leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
       // rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

        // Tank Mode uses one stick to control each wheel.
        // - This requires no math, but it is hard to drive forward slowly and keep straight.
        // leftPower  = -gamepad1.left_stick_y ;
        // rightPower = -gamepad1.right_stick_y ;

        // Send calculated power to wheels
       // leftDrive.setPower(leftPower);
        //rightDrive.setPower(rightPower);
       //SS leftDrive.setPower(rightPower);
       //SS rightDrive.setPower(leftPower);



        double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = gamepad1.right_stick_x;
        double v1 = r * Math.cos(robotAngle) + rightX;
        double v2 = r * Math.sin(robotAngle) - rightX;
        double v3 = r * Math.sin(robotAngle) + rightX;
        double v4 = r * Math.cos(robotAngle) - rightX;

        if (gamepad1.left_bumper == true) {
            sleep(100);
            if (speedFlag == true)
                speedFlag = false;
            else
                speedFlag = true;
            while(gamepad1.left_bumper == true);
        }

        if (speedFlag == true){
            if (v1>.20)
                v1=.20;
            if (v1<-.20)
                v1=-.20;
            if (v2>.20)
                v2=.20;
            if (v2<-.20)
                v2=-.20;
            if (v3>.20)
                v3=.20;
            if (v3<-.20)
                v3=-.20;
            if (v4>.20)
                v4=.20;
            if (v4<-.20)
                v4=-.20;
        }
        motorLF.setPower(v1);
        motorRF.setPower(v2);
        motorLB.setPower(v3);
        motorRB.setPower(v4);


        // Claw action


        if(gamepad1.right_bumper == true) {
            clawServo.setPosition(1);
        }
        else {
            clawServo.setPosition(0);
        }

        /*if(gamepad1.x){
            clawServo.setPosition(0.125);
        }
        //else{////////////////////////zaq
        /////clawServo.setPosition(0);/
        //}///////////////////////////
        if (gamepad1.b){
            clawServo.setPosition(0);
        }*/


        //Lift action
        if (gamepad1.y==true) {

            //liftServoRight.setPower(1.0);
            //liftServoLeft.setPower(-1.0);
            motorLift.setPower(1);
            motorLift1.setPower(1);
        }
        if (gamepad1.a==true){

            //liftServoRight.setPower(-1.0);
            //liftServoLeft.setPower(1.0);
            motorLift.setPower(-1);
            motorLift1.setPower(-1);
        }
        if (gamepad1.y==false && gamepad1.a==false){

            //liftServoRight.setPower(-0.07);
            //liftServoLeft.setPower(-0.07);
            motorLift.setPower(.01);
            motorLift1.setPower(.01);
        }
        //Reset the motorLift position to Zero at the bottom of the lift






        //Arm action
        if (gamepad1.dpad_up==true) {
            motorArm.setPower(1.0);
        }
        if (gamepad1.dpad_down==true){
                motorArm.setPower(-1);
        }
       /* if (gamepad1.dpad_down==true){
            if(motorArm.getCurrentPosition()-50 > 250){
                motorArm.setPower(-1);
            }
            else{
                motorArm.setPower(0);
            }

        }
        */
        if (gamepad1.dpad_up==false && gamepad1.dpad_down==false){

            motorArm.setPower(0);
        }

/*
        if(motorArm.getCurrentPosition() < 250){
            motorArm.setTargetPosition(250);
            motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorArm.setPower(Math.abs(1));
            motorArm.setPower(0);
            motorArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        if (gamepad1.start == true) {
            sleep(100);
            motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
     //       motorArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            while(gamepad1.start == true);
        }
*/
        //Telemety
        // Show the elapsed game time and wheel power.
        telemetry.addData("Name: ", "Sarvesh Robot");
       // telemetry.addData("Gamepad Status : ", "x (%.2f)", gamepad1.left_stick_x );
        //telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
       // telemetry.addData("Status", "Run Time: " + runtime.toString());
       // telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
        telemetry.addData("speedFlag", speedFlag);
        telemetry.addData("Gamepad1.start:",gamepad1.start);
        telemetry.addData("V1 :", v1);
        telemetry.addData("V2 :", v2);
        telemetry.addData("V3 :", v3);
        telemetry.addData("V4 :", v4);
        telemetry.addData("motorLift position :", motorLift.getCurrentPosition());
        telemetry.addData("motorLift1 Position :", motorLift1.getCurrentPosition());
        telemetry.addData("motorArm:", motorArm.getCurrentPosition());
        telemetry.addData("MotorArm reset:",motorArm.getCurrentPosition());
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        motorArm.setTargetPosition(10);
        motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorArm.setPower(-1);
       // motorArm.setPower(0);
        motorArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }

    }
