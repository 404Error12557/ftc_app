

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
 * This file contains an TeleOpMode (Non-Linear) Code for Team 12557.
 *
 */

@TeleOp(name="Basic: Iterative OpMode", group="Iterative Opmode")
//@Disabled
public class TeleOp12557 extends OpMode
{
    /*
    // Declare OpMode members.
    */

    //Timer variable for displaying telemerty
    private ElapsedTime runtime = new ElapsedTime();

    // Boolen variable for controling the speed of the Robot when left bumper switch is clicked.
    private boolean speedFlag = false;

    // Instance variable for DcMotors. This includes motor for 4 drivetrain motors, two lift motors and one arm motor
    private DcMotor motorRF = null; // Drivetrain Right Front motor
    private DcMotor motorRB = null; //Drivetrain Right Back motor
    private DcMotor motorLF = null; //Drivetrain Left Frount motor
    private DcMotor motorLB = null; //Drivetrain Left Back motor
   // private DcMotor motorLift = null; //Lift Motor
    //private DcMotor motorLift1 = null; // Lift Motor 1
    //private DcMotor motorArm = null; // Arm motor for tape measure

    // Instance variable for Servos
   // private Servo clawServo= null; // Claw Servo

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


        //clawServo = hardwareMap.get(Servo.class, "claw_servo");
        //clawServo.scaleRange(0.4,0.8);





        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery


        motorLF  = hardwareMap.get(DcMotor.class, "mLF");
        motorRF = hardwareMap.get(DcMotor.class, "mRF");
        motorLB = hardwareMap.get(DcMotor.class, "mLB");
        motorRB = hardwareMap.get(DcMotor.class, "mRB");
      // motorLift = hardwareMap.get(DcMotor.class, "mLift");
       // motorArm = hardwareMap.get(DcMotor.class, "mArm");
       // motorLift1 =hardwareMap.get(DcMotor.class, "mLift1");

       motorLF.setDirection(DcMotor.Direction.REVERSE);
        motorRB.setDirection(DcMotor.Direction.REVERSE);
       // motorArm.setDirection(DcMotor.Direction.REVERSE);

        //motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);//restsets the encoder
        //motorArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);//runs to use the encoder


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
q    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        /*
        // Macnum wheel code.
        */
        double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = gamepad1.right_stick_x;
        double v1 = r * Math.cos(robotAngle) + rightX;
        double v2 = r * Math.sin(robotAngle) - rightX;
        double v3 = r * Math.sin(robotAngle) + rightX;
        double v4 = r * Math.cos(robotAngle) - rightX;

        //Code for controling the speed when left bumper switch is pressed
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

        /*
        // Claw action
        */

       // if(gamepad1.right_bumper == true) {
      //      clawServo.setPosition(1);
    //    }
      //  else {
      //     clawServo.setPosition(0);
     //   }


<<<<<<< HEAD
        /*
        //Lift action
        */
      /*  if (gamepad1.y==true) {
=======
        //Lifts and lowers the arm up and down vertically
        if (gamepad1.y==true) {
>>>>>>> c05d997bf4fa6d5d782459c3a545ca4fd6d9154b

            // when the y button on the game pad is pressed
            motorLift.setPower(1);
            motorLift1.setPower(1);
            // lift the arm up by setting the motorlifts to 1 power
        }
        if (gamepad1.a==true){
            // when the a button on the gamepad is pressed
            motorLift.setPower(-1);
            motorLift1.setPower(-1);
            // lower the arm down by by setting the motorlifts to -1 power
        }
        if (gamepad1.y==false && gamepad1.a==false){
            motorLift.setPower(.01);
            motorLift1.setPower(.01);
        }
*/
        /*
        //Arm action
        */


        /*if (gamepad1.dpad_up==true) {
            motorArm.setPower(1.0);
        }
        if (gamepad1.dpad_down==true){
                motorArm.setPower(-1);
        }*/



       /* Do not remove this code
       if (gamepad1.dpad_down==true){
            if(motorArm.getCurrentPosition()-50 > 250){
                motorArm.setPower(-1);
            }
            else{
                motorArm.setPower(0);
            }

        }
        */
      /*  if (gamepad1.dpad_up==false && gamepad1.dpad_down==false){

            motorArm.setPower(0);
        }
*/


/*      Do not remove code.
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
        /*
        //Telemety
        */
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
        //telemetry.addData("motorLift position :", motorLift.getCurrentPosition());
        //telemetry.addData("motorLift1 Position :", motorLift1.getCurrentPosition());
        //telemetry.addData("motorArm:", motorArm.getCurrentPosition());
        //telemetry.addData("MotorArm reset:",motorArm.getCurrentPosition());
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        /*
        Do not remove this code
        motorArm.setTargetPosition(10);
        motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorArm.setPower(-1);
       // motorArm.setPower(0);
        motorArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        */

    }

    }
