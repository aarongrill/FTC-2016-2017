package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.LightSensor;
import android.graphics.Color;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;



/**
 * This file illustrates the concept of driving up to a line and then stopping.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code shows using two different light sensors:
 *   The Primary sensor shown in this code is a legacy NXT Light sensor (called "sensor_light")
 *   Alternative "commented out" code uses a MR Optical Distance Sensor (called "sensor_ods")
 *   instead of the LEGO sensor.  Chose to use one sensor or the other.
 *
 *   Setting the correct WHITE_THRESHOLD value is key to stopping correctly.
 *   This should be set half way between the light and dark values.
 *   These values can be read on the screen once the OpMode has been INIT, but before it is STARTED.
 *   Move the senso on asnd off the white line and not the min and max readings.
 *   Edit this code to make WHITE_THRESHOLD half way between the min and max.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="UltroAuto", group="Pushbot")
//@Disabled
public class UltroDriveToLine extends LinearOpMode {

    /* Declare OpMode members. */
    org.firstinspires.ftc.teamcode.HardwarePushbot robot       = new org.firstinspires.ftc.teamcode.HardwarePushbot();   // Use a Pushbot's hardware
    public ElapsedTime     runtime = new ElapsedTime();

    //LightSensor             lightSensor;      // Primary LEGO Light sensor,
    OpticalDistanceSensor opticalDistanceSensor;   // Alternative MR ODS sensor
    ColorSensor colorSensor;
    ModernRoboticsI2cGyro   gyro    = null;                    // Additional Gyro device
    
    static final double     WHITE_THRESHOLD = 0.3;  // spans between 0.1 - 0.5 from dark to light
    static final double     APPROACH_SPEED  = 0.5;
    
    //Gyro Setup
    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    //Gyro
    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double     DRIVE_SPEED             = 0.7;     // Nominal speed for better accuracy.
    static final double     TURN_SPEED              = 0.5;     // Nominal half speed for better accuracy.

    //Gyro
    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.15;     // Larger is more responsive, but also less stable
    
    @Override
    public void runOpMode() {

        /* Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        
        
        robot.init(hardwareMap);
        
        /* GYRO SETUP FROM HERE TO END GYRO SETUP */
       
        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");
        // Ensure the robot it stationary, then reset the encoders and calibrate the gyro.
        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Send telemetry message to alert driver that we are calibrating;
        telemetry.addData(">", "Calibrating Gyro");    //
        telemetry.update();

        gyro.calibrate();

        // make sure the gyro is calibrated before continuing
        while (!isStopRequested() && gyro.isCalibrating())  {
            sleep(50);
            idle();
        }

        telemetry.addData(">", "Robot Ready.");    //
        telemetry.update();

        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Wait for the game to start (Display Gyro value), and reset gyro before we move..
        while (!isStarted()) {
            telemetry.addData(">", "Robot Heading = %d", gyro.getIntegratedZValue());
            telemetry.update();
            idle();
        }
        gyro.resetZAxisIntegrator();
        
        /* END GYRO SETUP */
        
        /* SAMPLE
        gyroDrive(DRIVE_SPEED, 48.0, 0.0);    // Drive FWD 48 inches
        gyroTurn( TURN_SPEED, -45.0);         // Turn  CCW to -45 Degrees
        gyroHold( TURN_SPEED, -45.0, 0.5);    // Hold -45 Deg heading for a 1/2 second
        gyroTurn( TURN_SPEED,  45.0);         // Turn  CW  to  45 Degrees
        gyroHold( TURN_SPEED,  45.0, 0.5);    // Hold  45 Deg heading for a 1/2 second
        gyroTurn( TURN_SPEED,   0.0);         // Turn  CW  to   0 Degrees
        gyroHold( TURN_SPEED,   0.0, 1.0);    // Hold  0 Deg heading for a 1 second
        gyroDrive(DRIVE_SPEED,-48.0, 0.0);    // Drive REV 48 inches
        gyroHold( TURN_SPEED,   0.0, 0.5);    // Hold  0 Deg heading for a 1/2 second

        telemetry.addData("Path", "Complete");
        telemetry.update();
        
        */
        
        //Turn on flywheels for 3 seconds, then turn on elevator (intake b) for 5 seconds
        shootBalls(3, 5);

        runtime.reset();
        
        //turn in degrees 
        turn(-30);
        
        runtime.reset();

        driveToLine();

        //turnToBeacon();

        decideColor();

        press();
        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        // robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // get a reference to our Light Sensor object.
        opticalDistanceSensor = hardwareMap.opticalDistanceSensor.get("sensor_ods");
        colorSensor = hardwareMap.colorSensor.get("sensor_color");// Primary LEGO Light Sensor
        //  lightSensor = hardwareMap.opticalDistanceSensor.get("sensor_ods");  // Alternative MR ODS sensor.

        // turn on LED of light sensor.
        opticalDistanceSensor.enableLed(true);

        // Send telemetry message to signify robot waiting;
        //telemetry.addData("Status", "Ready to run");    //
        //telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        // Abort this loop if started or stopped.
        while (!(isStarted() || isStopRequested())) {

            // Display the light level while we are waiting to start
            telemetry.addData("Light Level", opticalDistanceSensor.getLightDetected());
            telemetry.update();
            idle();
        }

        // Start the robot moving forward, and then begin looking for a white line.
        /*robot.leftMotor.setPower(APPROACH_SPEED);
        robot.rightMotor.setPower(APPROACH_SPEED);

        // run until the white line is seen OR the driver presses STOP;
        while (opModeIsActive() && (opticalDistanceSensor.getLightDetected() < WHITE_THRESHOLD)) {

            // Display the light level while we are looking for the line
            telemetry.addData("Light Level",  opticalDistanceSensor.getLightDetected());
            telemetry.update();
        }*/

        // Stop all motors
        //robot.leftMotor.setPower(0);
        //robot.rightMotor.setPower(0);
    }

    public void shootBalls()
    {
        while(runtime.seconds()<= 5.0) {
            robot.elevatorMotor.setPower(1.0);
            robot.leftFlyWheel.setPower(0.25);
            robot.rightFlyWheel.setPower(-0.25);
        }
        robot.elevatorMotor.setPower(0.0);
        robot.leftFlyWheel.setPower(0.0);
        robot.rightFlyWheel.setPower(0.0);
    }

    public void turn(){
        while(runtime.seconds()<=1.0){
            robot.rightMotor.setPower(1.0);
            robot.leftMotor.setPower(-1.0);
        }
    }

    public void driveToLine()
    {
        // Start the robot moving forward, and then begin looking for a white line.
        robot.leftMotor.setPower(APPROACH_SPEED);
        robot.rightMotor.setPower(APPROACH_SPEED);

        telemetry.addData("Light Level",  opticalDistanceSensor.getLightDetected());
        telemetry.update();

        // run until the white line is seen OR the driver presses STOP;
        if (opticalDistanceSensor.getLightDetected() < WHITE_THRESHOLD) {

            // Display the light level while we are looking for the line
            robot.leftMotor.setPower(0.0);
            robot.leftMotor.setPower(0.0);
        }
    }

    public void turnToBeacon()
    {

    }

    public void followLine()
    {
        while (opModeIsActive() && (opticalDistanceSensor.getLightDetected() < WHITE_THRESHOLD))
        {
            robot.leftMotor.setPower(APPROACH_SPEED);
            robot.rightMotor.setPower(APPROACH_SPEED);
        }
    }

    public void decideColor()
    {

        float hsvValues[] = {0,0,0};

        colorSensor.enableLed(false);

        Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);

        telemetry.addData("2 Clear", colorSensor.alpha());
        telemetry.addData("3 Red", colorSensor.red());
        telemetry.addData("4 Green", colorSensor.green());
        telemetry.addData("5 Blue", colorSensor.blue());
        telemetry.addData("6 Hue", hsvValues[0]);

        if(colorSensor.red() > colorSensor.blue() && colorSensor.red() > colorSensor.green())
        {
            robot.rightPad.setPosition(0.0);
            robot.leftPad.setPosition(0.5);
        }
    }

    public void press(){
        while (opModeIsActive() && runtime.seconds()<=2.0) {
            robot.leftMotor.setPower(APPROACH_SPEED);
            robot.rightMotor.setPower(APPROACH_SPEED);
        }
    }
