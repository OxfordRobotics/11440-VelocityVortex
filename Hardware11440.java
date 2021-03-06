package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class Hardware11440
{
    /* Public OpMode members. */
    public DcMotor  leftMotor             = null;
    public DcMotor  rightMotor            = null;
    public DcMotor  leftBackMotor         = null;
    public DcMotor  rightBackMotor        = null;
    public DcMotor  leftFrontMotor        = null;
    public DcMotor  rightFrontMotor       = null;
    public DcMotor  kickMotor             = null;
    public DcMotor  sweepMotor            = null;
    public DcMotor  flyRight              = null;
    public DcMotor  flyLeft               = null;
    public ColorSensor frontCS            = null;
    public ColorSensor bbtmCS             = null;
    public ColorSensor fbtmCS             = null;
    public OpticalDistanceSensor ods      = null;

    /*public Servo    leftClaw    = null;
    public Servo    rightClaw   = null;
    public static final double MID_SERVO       =  0.5 ;
    public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public Hardware11440(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        //servo value 0-255
        leftBackMotor   = hwMap.dcMotor.get("bleft_drive");
        rightBackMotor  = hwMap.dcMotor.get("bright_drive");
        rightFrontMotor = hwMap.dcMotor.get("fright_drive");
        leftFrontMotor  = hwMap.dcMotor.get("fleft_drive");
        kickMotor       = hwMap.dcMotor.get("kick_motor");
        sweepMotor      = hwMap.dcMotor.get("sweep_motor");
        flyRight        = hwMap.dcMotor.get("right_fly");
        flyLeft         = hwMap.dcMotor.get("left_fly");
        frontCS         = hwMap.colorSensor.get("front_cs");
        bbtmCS          = hwMap.colorSensor.get("bbtm_cs");
        fbtmCS          = hwMap.colorSensor.get("fbtm_cs");
        ods             = hwMap.opticalDistanceSensor.get("dis_sensor");

        /*leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        // Set all motors to zero power
        leftMotor.setPower(0);
        rightMotor.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.
        //leftClaw = hwMap.servo.get("left_hand");
        //rightClaw = hwMap.servo.get("right_hand");
        //leftClaw.setPosition(MID_SERVO);
        //rightClaw.setPosition(MID_SERVO);
    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     * @throws InterruptedException
     *
    public void waitForTick(long periodMs) throws InterruptedException {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            Thread.sleep(remaining);

        // Reset the cycle clock for the next pass.
        period.reset();*/
    }
}

