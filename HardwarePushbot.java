package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;
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
public class HardwarePushbot {
    /* Public OpMode members. */
    public DcMotor  leftMotor   = null;
    public DcMotor  rightMotor  = null;
    public DcMotor  intakeAMotor    = null;
    public DcMotor  intakeBMotor  = null;
    public DcMotor flyWheelLeftMotor  = null;
    public DcMotor flyWheelRightMotor = null;
    public Servo    leftBeacon    = null;
    public Servo    rightBeacon   = null;
    public ColorSensor colorSensor = null;
    public OpticalDistanceSensor opticalDistanceSensor = null;
    public ModernRoboticsI2cGyro gyro = null;
    public static final double MID_SERVO       =  0.5 ;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwarePushbot(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftMotor   = hwMap.dcMotor.get("left_drive");
        rightMotor  = hwMap.dcMotor.get("right_drive");
        flyWheelLeftMotor    = hwMap.dcMotor.get("fly_Left");
        flyWheelRightMotor   = hwMap.dcMotor.get("fly_Right");
        intakeAMotor   = hwMap.dcMotor.get("intake_a");
        intakeBMotor   = hwMap.dcMotor.get("intake_b");
        opticalDistanceSensor = hwMap.opticalDistanceSensor.get("sensor_ods");
        colorSensor = hwMap.colorSensor.get("sensor_color");
        gyro = (ModernRoboticsI2cGyro)hwMap.gyroSensor.get("sensor_gyro");

        leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        intakeAMotor.setDirection(DcMotor.Direction.FORWARD);
        intakeBMotor.setDirection(DcMotor.Direction.FORWARD);
        flyWheelLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        flyWheelRightMotor.setDirection(DcMotor.Direction.REVERSE);
        // Set all motors to zero power
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        intakeAMotor.setPower(0);
        intakeBMotor.setPower(0);
        flyWheelLeftMotor.setPower((0));
        flyWheelRightMotor.setPower(0);
        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeAMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeBMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flyWheelLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flyWheelRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Define and initialize ALL installed servos.
        leftBeacon = hwMap.servo.get("left_Beacon");
        rightBeacon = hwMap.servo.get("right_Beacon");
        leftBeacon.setPosition(MID_SERVO);
        rightBeacon.setPosition(MID_SERVO);
    }
    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     * @throws InterruptedException
     */
    public void waitForTick(long periodMs) throws InterruptedException {
        long  remaining = periodMs - (long)period.milliseconds();
        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            Thread.sleep(remaining);
        // Reset the cycle clock for the next pass.
        period.reset();
    }
}
