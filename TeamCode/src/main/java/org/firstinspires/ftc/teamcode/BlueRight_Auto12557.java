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

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;


@Autonomous(name="Blue Right - Auto", group="Autonomous")
//@Disabled
public class BlueRight_Auto12557 extends LinearOpMode {
    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;
    RelicRecoveryVuMark vuMark;
    /** The colorSensor field will contain a reference to our color sensor hardware object */
    NormalizedColorSensor colorSensor;

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    private boolean speedFlag = false;

    private DcMotor motorRF = null;
    private DcMotor motorRB = null;
    private DcMotor motorLF = null;
    private DcMotor motorLB = null;
    private DcMotor motorLift1 = null;
    private DcMotor motorArm = null;


    private Servo LeftJewelServo= null;
    private Servo RightJewelServo = null;
    private Servo clawServo= null;
    private Servo colorRedServo = null;
    private Servo colorBlueServo = null;

    OpenGLMatrix lastLocation = null;

    //static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     COUNTS_PER_MOTOR_REV    = 1120;  // eg: Neverest Motor Encoder
    static final double     COUNTS_PER_COREX_MOTOR_REV  = 288 ;// THE REV COREX MOTORS
    static final double     ARMWHEEL_DIAMETER_INCHES =1.5 ; // THE WHEEL ON THE LINEAR SLIDE
    static final double     SHAFT_DIAMETER  = 1; //0.024; // THE DIAMETER OF THE SHAFT IN INCHES
    static final double     DRIVE_GEAR_REDUCTION    = 1 ;     // This is < 1.0 if geared UP
    static final double     DRIVE_GEAR_REDUCTION_COREX   = 1 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     ARMWHEEL_COUNTS_PER_INCH    = (COUNTS_PER_COREX_MOTOR_REV * DRIVE_GEAR_REDUCTION_COREX) /
                                                                (ARMWHEEL_DIAMETER_INCHES * 3.1415);
    static final double     SHAFT_COUNTS_PER_INCH    = (COUNTS_PER_COREX_MOTOR_REV * DRIVE_GEAR_REDUCTION_COREX) /
                                                                (SHAFT_DIAMETER * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    static final double     DRIVE_FORWARD              = 0;
    static final double     DRIVE_BACKWORD             = 1;
    static final double     DRIVE_TURN_LEFT            = 2;
    static final double     DRIVE_TURN_RIGHT           = 3;
    static final double     DRIVE_STRAFE_lEFT          = 4;
    static final double     DRIVE_STRAFE_RIGHT         = 5;
    static final double     ARM_EXTEND                 = 6;
    static final double     LIFT_UP                    = 7;
    static final double     LIFT_DOWN                  = 8;

    static final double     VUMARK_LEFT =1;
    static final double     VUMARK_CENTER =2;
    static final double     VUMARK_RIGHT =3;



    @Override
    public void runOpMode()throws InterruptedException {

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Version", 9);



        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */

        // Get a reference to our sensor object.
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
        boolean red = false;

        // If possible, turn the light on in the beginning (it might already be on anyway,
        // we just make sure it is if we can).
        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight)colorSensor).enableLight(false);
        }
        /*
         * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
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
         * and paste it in to your code onthe next line, between the double quotes.
         */
        parameters.vuforiaLicenseKey = "AbtLkrX/////AAAAGQlccZwEI0u/l2eq6fmFVstmH4Bhae2L6Z+FYWLECuufmOj/KQCF6vGj8Ys7x644Xqro8fTkpzQ0m3vI8k9tANQaPf7aqzefRBicfr1I6YbwoHKDKrBUsjjG8thlsOWgMLG4uBcAknaTJjGwW4d8F3eBeKjAv3d65tMTV5bZo8y6q+xx5MlTCvXR5YcJBP2rOpNVP9HwpDWEDs0+0WH2ZfWmOp0F4uQ6SNtUgj5nd4AcGP/btXkkU/f3FknFeEM0zmoBcaSQYZvkT0Rayv76B/yKsucOTuam4Tkbpzocw6AcmO4QQymw68x0bzJRhnpK2o5PxnhrSRmT1ueRsHKOXrxw1EFr5mrU7FrP5NfDbhwM";
      /*
         * We also indicate which camera on the RC that we wish to use.
         * Here we chose the back (HiRes) camera (for greater range), but
         * for a competition robot, the front camera might be more convenient.
         */
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        /**
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one template,
         * but differ in their instance id information.
         * @see VuMarkInstanceId
         */
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        double direction=0;
        int i =0;

        clawServo = hardwareMap.get(Servo.class, "claw_servo");
        clawServo.scaleRange(0.42,0.8);

     //   colorRedServo = hardwareMap.get(Servo.class, "color_red_servo");
        //colorRedServo.setDirection(Servo.Direction.REVERSE);
       // colorRedServo.scaleRange(0,0.6);
        colorBlueServo = hardwareMap.get(Servo.class, "color_blue_servo");
       // colorBlueServo.scaleRange(0.48,0.8);


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery


        motorLF  = hardwareMap.get(DcMotor.class, "mLF");
        motorRF = hardwareMap.get(DcMotor.class, "mRF");
        motorLB = hardwareMap.get(DcMotor.class, "mLB");
        motorRB = hardwareMap.get(DcMotor.class, "mRB");
        motorLift1 = hardwareMap.get(DcMotor.class, "mLift1");
        motorArm = hardwareMap.get(DcMotor.class, "mArm");

        motorLF.setDirection(DcMotor.Direction.REVERSE);
        motorRB.setDirection(DcMotor.Direction.REVERSE);
        motorLift1.setDirection(DcMotor.Direction.REVERSE);
        colorBlueServo.setDirection(Servo.Direction.REVERSE);
        motorLift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //


        //Close the claw to hold glyph when init button is pressed but before play button is pressed.
        clawServo.setPosition(1);
        colorBlueServo.setPosition(.3);
        relicTrackables.activate();

            /**
             * See if any of the instances of {@link relicTemplate} are currently visible.
             * {@link RelicRecoveryVuMark} is an enum which can have the following values:
             * UNKNOWN, LEFT, CENTER, and RIGHT. When a VuMark is visible, something other than
             * UNKNOWN will be returned by {@link RelicRecoveryVuMark#from(VuforiaTrackable)}.
             */

            while (i!=3) {
                vuMark = RelicRecoveryVuMark.from(relicTemplate);
                sleep(1000);
                if (vuMark != RelicRecoveryVuMark.UNKNOWN && vuMark != null) {
                    i = 3;
                }
                else
                {
                    i = i + 1;
                }
            }
            if (vuMark != RelicRecoveryVuMark.UNKNOWN && vuMark != null) {

                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */
                telemetry.addData("VuMark", "%s visible", vuMark);
                if(vuMark ==RelicRecoveryVuMark.LEFT )
                    direction = -7;

                if(vuMark ==RelicRecoveryVuMark.CENTER )
                    direction = -1;

                if(vuMark ==RelicRecoveryVuMark.RIGHT )
                    direction = 7;

            }
            else {
                telemetry.addData("VuMark", "not visible");
            }



        telemetry.addData("direction: ",  direction );
        telemetry.addData("opModeIsActive: ",  opModeIsActive() );

        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        colorBlueServo.setPosition(.7);
        sleep(2000);
        // Read the sensor
        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight)colorSensor).enableLight(true);
        }

        NormalizedRGBA colors =  colorSensor.getNormalizedColors();
 /*       i=0;
        while (i!=3) {
            colors =  colorSensor.getNormalizedColors();
            sleep(1000);
           i=i+1;
        }
        */
        // Balance the colors. The values returned by getColors() are normalized relative to the
        // maximum possible values that the sensor can measure. For example, a sensor might in a
        // particular configuration be able to internally measure color intensity in a range of
        // [0, 10240]. In such a case, the values returned by getColors() will be divided by 10240
        // so as to return a value it the range [0,1]. However, and this is the point, even so, the
        // values we see here may not get close to 1.0 in, e.g., low light conditions where the
        // sensor measurements don't approach their maximum limit. In such situations, the *relative*
        // intensities of the colors are likely what is most interesting. Here, for example, we boost
        // the signal on the colors while maintaining their relative balance so as to give more
        // vibrant visual feedback on the robot controller visual display.
        float max = Math.max(Math.max(Math.max(colors.red, colors.green), colors.blue), colors.alpha);
        colors.red   /= max;
        colors.green /= max;
        colors.blue  /= max;
        int color = colors.toColor();

        if ( Color.red(color)>Color.blue(color))
        {
            red = true;
            telemetry.addData("color: ", "Red");
        }
        else if(Color.blue(color)>Color.red(color)) {
            telemetry.addData("color: ", "Blue");
        }
        else
        {
            telemetry.addData("color: ", "Green");
        }
        telemetry.addData("red: " , red);

 /*       telemetry.addLine("normalized color:  ")
                .addData("a", "%02x", Color.alpha(color))
                .addData("r", "%02x", Color.red(color))
                .addData("g", "%02x", Color.green(color))
                .addData("b", "%02x", Color.blue(color));
                */
        telemetry.update();
        //sleep(5000);
        if (!red) {
            encoderDrive(DRIVE_SPEED, DRIVE_TURN_LEFT, 30, 5.0);  // turn right. 3rd parameter is degree of turn. eg 45degree or 90degree
            colorBlueServo.setPosition(0);
            sleep(1000);
            encoderDrive(DRIVE_SPEED, DRIVE_TURN_RIGHT
                    , 30, 5.0);  // turn right. 3rd parameter is degree of turn. eg 45degree or 90degree
        }
        else
        {
            encoderDrive(DRIVE_SPEED, DRIVE_TURN_RIGHT, 30, 5.0);  // turn right. 3rd parameter is degree of turn. eg 45degree or 90degree
            colorBlueServo.setPosition(0);
            sleep(1000);
            encoderDrive(DRIVE_SPEED, DRIVE_TURN_LEFT, 30, 5.0);  // turn right. 3rd parameter is degree of turn. eg 45degree or 90degree
        }

        colorBlueServo.setPosition(0);


        // calling the encoderDrive to driver motors (macnum wheels, lift and arm motors).
        /***********examples (do not delete)*************
         encoderDrive(DRIVE_SPEED, DRIVE_FORWARD, 366, 5.0);  // Forward. 3rd parameter is how many inches you want robot to move forward.
         encoderDrive(DRIVE_SPEED, DRIVE_BACKWORD, 24, 5.0);  // backwards. 3rd parameter is how many inches you want robot to move backword.
         encoderDrive(DRIVE_SPEED, DRIVE_TURN_LEFT, 45, 5.0);  // turn left. 3rd parameter is degree of turn. eg 45degree or 90degree
         encoderDrive(DRIVE_SPEED, DRIVE_TURN_RIGHT, 90, 5.0);  // turn right. 3rd parameter is degree of turn. eg 45degree or 90degree
         encoderDrive(DRIVE_SPEED, DRIVE_STRAFE_lEFT, 24, 5.0);  // left strafe. 3rd parameter is how many inches you want robot to move left.
         encoderDrive(DRIVE_SPEED, DRIVE_STRAFE_RIGHT, 24, 5.0);  // right strafe. 3rd parameter is how many inches you want robot to move right.
         encoderDrive(DRIVE_SPEED, ARM_EXTEND, 5, 5.0 ); // not completed yet.
         encoderDrive(DRIVE_SPEED, LIFT_UP, 5, 5.0); // not completed yet.
       */
        encoderDrive(DRIVE_SPEED, DRIVE_FORWARD, 35+direction, 5.0);  // S3: Forward
        encoderDrive(DRIVE_SPEED, DRIVE_TURN_LEFT, 90,  5.0);  // S3: right
        encoderDrive(DRIVE_SPEED, DRIVE_FORWARD, 8,   5.0);  // S3: Forward
        //Open the claw to release the glyph
        clawServo.setPosition(0);
        sleep(1000);   // optional pause after each move
        encoderDrive(DRIVE_SPEED, DRIVE_BACKWORD, 8, 5.0);  // backwards. 3rd parameter is how many inches you want robot to move backword.
         // sleep(10000);     // pause for servos to move
        clawServo.setPosition(1);
        encoderDrive(DRIVE_SPEED, DRIVE_FORWARD, 6, 5.0);  // backwards. 3rd parameter is how many inches you want robot to move backword.
        encoderDrive(DRIVE_SPEED, DRIVE_BACKWORD, 6, 5.0);  // backwards. 3rd parameter is how many inches you want robot to move backword.
        clawServo.setPosition(0);


/*
        telemetry.addLine("Complete");
        telemetry.update();
        */
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
        int newRightBackTarget = 0;
        int newArmPostion = 0;
        int newLift1Position = 0;

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
                newLeftFrontTarget = motorLF.getCurrentPosition() + (int) ((-distanceToTravel/ 4.2) * COUNTS_PER_INCH);
                newLeftBackTarget = motorLB.getCurrentPosition() + (int) ((distanceToTravel/ 4.2) * COUNTS_PER_INCH);
                newRightFrontTarget = motorRF.getCurrentPosition() + (int) ((distanceToTravel/ 4.2) * COUNTS_PER_INCH);
                newRightBackTarget = motorRB.getCurrentPosition() + (int) ((-distanceToTravel/ 4.2) * COUNTS_PER_INCH);
            }
            if (direction== DRIVE_TURN_RIGHT) {
                newLeftFrontTarget = motorLF.getCurrentPosition() + (int) ((distanceToTravel / 4.2) * COUNTS_PER_INCH);
                newLeftBackTarget = motorLB.getCurrentPosition() + (int) ((-distanceToTravel / 4.2) * COUNTS_PER_INCH);
                newRightFrontTarget = motorRF.getCurrentPosition() + (int) ((-distanceToTravel / 4.2) * COUNTS_PER_INCH);
                newRightBackTarget = motorRB.getCurrentPosition() + (int) ((distanceToTravel / 4.2) * COUNTS_PER_INCH);
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
            if(direction== LIFT_UP){
                 newLift1Position = motorLift1.getCurrentPosition() +(int) (distanceToTravel * SHAFT_COUNTS_PER_INCH);
            }
            if(direction== LIFT_DOWN){
                 newLift1Position = motorLift1.getCurrentPosition() +(int) (-distanceToTravel * SHAFT_COUNTS_PER_INCH);
            }
            if(direction== ARM_EXTEND){
                newArmPostion = motorArm.getCurrentPosition() +(int) (distanceToTravel * ARMWHEEL_COUNTS_PER_INCH);
            }
            motorLF.setTargetPosition(newLeftFrontTarget);
            motorLB.setTargetPosition(newLeftBackTarget);
            motorRF.setTargetPosition(newRightFrontTarget);
            motorRB.setTargetPosition(newRightBackTarget);
            motorLift1.setTargetPosition(newLift1Position);

            motorArm.setTargetPosition(newArmPostion);


            // Turn On RUN_TO_POSITION
            motorLF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorLB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorRF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorRB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorLift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();

            motorLF.setPower(Math.abs(speed));
            motorLB.setPower(Math.abs(speed));
            motorRF.setPower(Math.abs(speed));
            motorRB.setPower(Math.abs(speed));
            motorArm.setPower(Math.abs(speed));
            motorLift1.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (motorLF.isBusy() || motorLB.isBusy() || motorRF.isBusy() || motorRB.isBusy() || motorArm.isBusy() || motorLift1.isBusy())) {

                // Display it for the driver.
/*
                telemetry.addData("Path1",  "Running to %7d :%7d :%7d :%7d :%7d :%7d", newLeftFrontTarget, newLeftBackTarget, newRightFrontTarget, newRightBackTarget,newLift1Position, newArmPostion );
                telemetry.addData("Path2",  "Running at %7d :%7d :%7d :%7d :%7d :%7d",
                                            motorLF.getCurrentPosition(),
                                            motorLB.getCurrentPosition(),
                                            motorRF.getCurrentPosition(),
                                            motorRB.getCurrentPosition(),
                                            motorLift1.getCurrentPosition(),
                                            motorArm.getCurrentPosition());

                telemetry.update();
                */
            }

            // Stop all motion;

            motorLB.setPower(0);
            motorLF.setPower(0);
            motorRF.setPower(0);
            motorRB.setPower(0);
            motorLift1.setPower(0.1);
            motorArm.setPower(0);

            // Turn off RUN_TO_POSITION
            motorLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorLB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorRB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorLift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

           //   sleep(300);   // optional pause after each move
        }
    }


}
