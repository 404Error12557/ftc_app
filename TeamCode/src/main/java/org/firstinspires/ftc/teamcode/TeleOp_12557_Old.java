package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
 */

@TeleOp(name="TeleOp 12557 ", group="12557 Opmodes")


public class TeleOp_12557_Old extends OpMode
{
    // Declare contents or objects in OpMode
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor mtrLB = null;
    private DcMotor mtrRB = null;
    private DcMotor mtrLift = null;
    private Servo srvClawL =null;
    private Servo srvClawR =null;
    private boolean holdArm =false;

    // code will run when the driver his the init button on the driver station
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        /*Initialize the hardware variables declared earlier (Note that the strings used here as parameters
        to 'get' must correspond to the names assigned during the robot configuration on the RC phone*/
        mtrLB  = hardwareMap.get(DcMotor.class, "mtrLB");
        mtrRB = hardwareMap.get(DcMotor.class, "mtrRB");
        mtrLift = hardwareMap.get (DcMotor.class, "mtrlift");

        srvClawL = hardwareMap.get (Servo.class , "srvClawL");
        srvClawR = hardwareMap.get (Servo.class , "srvClawR");
        // Most robots need the motor on one side to be reversed to drive forward accurately
        // Reverse the motor that runs backwards when connected directly to the battery and Expansion Hub is turned on
        mtrLB.setDirection(DcMotor.Direction.FORWARD);
        mtrRB.setDirection(DcMotor.Direction.REVERSE);
        mtrLift.setDirection(DcMotor.Direction.FORWARD);

        srvClawL.setDirection(Servo.Direction.FORWARD);
        srvClawR.setDirection(Servo.Direction.REVERSE);

        // Tells the driver that initialization is complete with input on the DS phone
        telemetry.addData("Status", "Initialized");
    }

    /*Code in the init loop will run REPEATEDLY after the driver hits init, but before they hit the
    play button*/
    @Override
    public void init_loop() {
     // not needed for our program
    }

    /* Code to will run once now when the driver hits the play button and runtime resets the time it
     has been running*/
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        /////////////////this is for motors drive\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
        ////////////////////////////////////////////////////////////////////////
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower;
        double rightPower;

        // POV Mode uses left stick to go forward and turn.
        double drive = gamepad1.left_stick_y;
        double turn  =  -gamepad1.left_stick_x;
        leftPower    = Range.clip(drive - turn, -1.0, 1.0) ;
        rightPower   = Range.clip(drive + turn, -1.0, 1.0);

        // Send calculated power to wheels
        mtrLB.setPower(leftPower);
        mtrRB.setPower(rightPower);

        ////////////////////////////FOLLOWING CODE CONTROLS ARM//////////////////////////////////
        /*lifts the arm and range.clip restricts the range of motion for the arm so the motor doesn't
        over work and burn out;
        we have set the power of the arm motor to move when the gamepad 1 right stick is moved up
        and down along the y axis except on the gamepad y is when the stick is moved down along the
        y axis so we have to set the gamepad value to -y so when you move the stick forward the arm
        goes up*/
        mtrLift.setPower(Range.clip(-gamepad1.right_stick_y, -1.0,1.0)/2);


        /////////////////this is for servo claw\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
        ////////////////////////////////////////////////////////////////////////
    /*    if (gamepad1.right_bumper){
            if (holdArm == true){
                holdArm = false;
            }
          else{
                holdArm = true;
            }
            sleep(200);
        }
        */
        if (gamepad1.right_bumper){
            srvClawL.setPosition(0);
            srvClawR.setPosition(0);
        }else{
            srvClawL.setPosition(1);
            srvClawR.setPosition(1);



        }


        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
