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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;



@Autonomous(name="Pushbot: Auto Drive By Encoder", group="Pushbot")
//@Disabled
public class BlueLeftAuto extends LinearOpMode {

    /* Declare OpMode members. */


    private ElapsedTime runtime = new ElapsedTime();
    private boolean speedFlag = false;


    private DcMotor motorRF = null;
    private DcMotor motorRB = null;
    private DcMotor motorLF = null;
    private DcMotor motorLB = null;
    private DcMotor motorLift = null;
    private DcMotor motorArm = null;


    private Servo LeftJewelServo= null;
    private Servo RightJewelServo = null;
    private Servo clawServo= null;



    //static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     COUNTS_PER_MOTOR_REV    = 2240;  // eg: Neverest Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = .5 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    static final double     DRIVE_FORWARD              = 0;
    static final double     DRIVE_BACKWORD              = 1;
    static final double     DRIVE_TURN_LEFT              = 2;
    static final double     DRIVE_TURN_RIGHT              = 3;
    static final double     DRIVE_STRAFE_lEFT              = 4;
    static final double     DRIVE_STRAFE_RIGHT              = 5;




    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */

        clawServo = hardwareMap.get(Servo.class, "claw_servo");
        clawServo.scaleRange(0.48,0.8);


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery


        motorLF  = hardwareMap.get(DcMotor.class, "mLF");
        motorRF = hardwareMap.get(DcMotor.class, "mRF");
        motorLB = hardwareMap.get(DcMotor.class, "mLB");
        motorRB = hardwareMap.get(DcMotor.class, "mRB");
        motorLift = hardwareMap.get(DcMotor.class, "mLift");
        motorArm = hardwareMap.get(DcMotor.class, "mArm");

        motorLF.setDirection(DcMotor.Direction.REVERSE);
        motorRB.setDirection(DcMotor.Direction.REVERSE);
        // Tell the driver that initialization is complete.

        motorLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        motorLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("Status", "Initialized");


        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        encoderDrive(DRIVE_SPEED, DRIVE_FORWARD, 24, 5.0);  // S3: Forward
        encoderDrive(DRIVE_SPEED, DRIVE_BACKWORD, 24, 5.0);  // S3: backwards
        encoderDrive(DRIVE_SPEED, DRIVE_TURN_LEFT, 24, 5.0);  // S3: left
        encoderDrive(DRIVE_SPEED, DRIVE_TURN_RIGHT, 24, 5.0);  // S3: right
        encoderDrive(DRIVE_SPEED, DRIVE_STRAFE_lEFT, 24, 5.0);  // S3: right
        encoderDrive(DRIVE_SPEED, DRIVE_STRAFE_RIGHT, 24, 5.0);  // S3: right


        //robot.leftClaw.setPosition(1.0);            // S4: Stop and close the claw.
       // robot.rightClaw.setPosition(0.0);
        sleep(1000);     // pause for servos to move

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */

    public void encoderDrive(double speed,
                             double direction, double distanceToTravel,
                             double timeoutS)
    {

        int newLeftFrontTarget = 0;
        int newLeftBackTarget = 0;
        int newRightFrontTarget = 0;
        int newRightBackTarget = 0              ;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            if (direction== DRIVE_FORWARD) {
                newLeftFrontTarget = motorLF.getCurrentPosition() + (int) (-distanceToTravel * COUNTS_PER_INCH);
                newLeftBackTarget = motorLB.getCurrentPosition() + (int) (-distanceToTravel * COUNTS_PER_INCH);
                newRightFrontTarget = motorRF.getCurrentPosition() + (int) (-distanceToTravel * COUNTS_PER_INCH);
                newRightBackTarget = motorRB.getCurrentPosition() + (int) (-distanceToTravel * COUNTS_PER_INCH);
            }
            if (direction== DRIVE_BACKWORD) {
                newLeftFrontTarget = motorLF.getCurrentPosition() + (int) (distanceToTravel * COUNTS_PER_INCH);
                newLeftBackTarget = motorLB.getCurrentPosition() + (int) (distanceToTravel * COUNTS_PER_INCH);
                newRightFrontTarget = motorRF.getCurrentPosition() + (int) (distanceToTravel * COUNTS_PER_INCH);
                newRightBackTarget = motorRB.getCurrentPosition() + (int) (distanceToTravel * COUNTS_PER_INCH);
            }
            if (direction== DRIVE_TURN_LEFT) {
                newLeftFrontTarget = motorLF.getCurrentPosition() + (int) (-distanceToTravel * COUNTS_PER_INCH);
                newLeftBackTarget = motorLB.getCurrentPosition() + (int) (distanceToTravel * COUNTS_PER_INCH);
                newRightFrontTarget = motorRF.getCurrentPosition() + (int) (distanceToTravel * COUNTS_PER_INCH);
                newRightBackTarget = motorRB.getCurrentPosition() + (int) (-distanceToTravel * COUNTS_PER_INCH);
            }
            if (direction== DRIVE_TURN_RIGHT) {
                newLeftFrontTarget = motorLF.getCurrentPosition() + (int) (distanceToTravel * COUNTS_PER_INCH);
                newLeftBackTarget = motorLB.getCurrentPosition() + (int) (-distanceToTravel * COUNTS_PER_INCH);
                newRightFrontTarget = motorRF.getCurrentPosition() + (int) (-distanceToTravel * COUNTS_PER_INCH);
                newRightBackTarget = motorRB.getCurrentPosition() + (int) (distanceToTravel * COUNTS_PER_INCH);
            }
            if (direction== DRIVE_STRAFE_lEFT) {
                newLeftFrontTarget = motorLF.getCurrentPosition() + (int) (-distanceToTravel * COUNTS_PER_INCH);
                newLeftBackTarget = motorLB.getCurrentPosition() + (int) (-distanceToTravel * COUNTS_PER_INCH);
                newRightFrontTarget = motorRF.getCurrentPosition() + (int) (distanceToTravel * COUNTS_PER_INCH);
                newRightBackTarget = motorRB.getCurrentPosition() + (int) (distanceToTravel * COUNTS_PER_INCH);
            }
            if (direction== DRIVE_STRAFE_RIGHT) {
                newLeftFrontTarget = motorLF.getCurrentPosition() + (int) (distanceToTravel * COUNTS_PER_INCH);
                newLeftBackTarget = motorLB.getCurrentPosition() + (int) (distanceToTravel * COUNTS_PER_INCH);
                newRightFrontTarget = motorRF.getCurrentPosition() + (int) (-distanceToTravel * COUNTS_PER_INCH);
                newRightBackTarget = motorRB.getCurrentPosition() + (int) (-distanceToTravel * COUNTS_PER_INCH);
            }

            motorLF.setTargetPosition(newLeftFrontTarget);
            motorLB.setTargetPosition(newLeftBackTarget);
            motorRF.setTargetPosition(newRightFrontTarget);
            motorRB.setTargetPosition(newRightBackTarget);


            // Turn On RUN_TO_POSITION


            motorLF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorLB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorRF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorRB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();


            motorLF.setPower(Math.abs(speed));
            motorLB.setPower(Math.abs(speed));
            motorRF.setPower(Math.abs(speed));
            motorRB.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (motorLF.isBusy() && motorLB.isBusy() && motorRF.isBusy() && motorRB.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d :%7d :%7d ", newLeftFrontTarget, newLeftBackTarget, newRightFrontTarget, newRightBackTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d :%7d :%7d",
                                            motorLF.getCurrentPosition(),
                                            motorLB.getCurrentPosition(),
                                            motorRF.getCurrentPosition(),
                                            motorRB.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;

            motorLB.setPower(0);
            motorLF.setPower(0);
            motorRF.setPower(0);
            motorRB.setPower(0);

            // Turn off RUN_TO_POSITION


            motorLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorLB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorRB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
}
