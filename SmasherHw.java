package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
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
    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwarePushbot(){

    }

    /* Public OpMode members. */
    public DcMotor  leftFrontMotor   = null;
    public DcMotor  leftBackMotor  = null;
    public DcMotor  rightFrontMotor = null;
    public DcMotor rightBackMotor = null;
    public DcMotor intakeMotor = null;
    public DcMotor flyWheelMotor = null;

    public Servo leftBeacon = null;
    public Servo rightBeacon = null;
    public Servo limiter = null;

    public OpticalDistanceSensor odsSensorI;  // Hardware Device Object
    public OpticalDistanceSensor odsSensorII;
    public OpticalDistanceSensor odsSensorIII;
    public OpticalDistanceSensor odsSensorIV;

    public ModernRoboticsI2cGyro   gyro = null;

    public ColorSensor color;

    // public ModernRoboticsI2cRangeSensor range;


    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors and Sensors
        leftFrontMotor = hwMap.dcMotor.get("leftFront");
        leftBackMotor = hwMap.dcMotor.get("leftBack");
        rightFrontMotor = hwMap.dcMotor.get("rightFront");
        rightBackMotor = hwMap.dcMotor.get("rightBack");
        intakeMotor = hwMap.dcMotor.get("intake");
        flyWheelMotor = hwMap.dcMotor.get("flyWheel");
        odsSensorI = hwMap.opticalDistanceSensor.get("odsI");
        odsSensorII = hwMap.opticalDistanceSensor.get("odsII");
        odsSensorIII = hwMap.opticalDistanceSensor.get("odsIII");
        odsSensorIV = hwMap.opticalDistanceSensor.get("odsIV");
        leftBeacon = hwMap.servo.get("leftBeacon");
        rightBeacon = hwMap.servo.get("rightBeacon");
        limiter = hwMap.servo.get("limiter");
        gyro = (ModernRoboticsI2cGyro)hwMap.gyroSensor.get("gyro");
        color = hwMap.colorSensor.get("color");


        // Set Motor Direction
        leftFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        leftBackMotor.setDirection(DcMotor.Direction.FORWARD);
        rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        rightBackMotor.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        flyWheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //Set all motor powers to zero
        leftFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightFrontMotor.setPower(0);
        rightBackMotor.setPower(0);
        intakeMotor.setPower(0);
        //Establish whether or not you're using encoders
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flyWheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



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
