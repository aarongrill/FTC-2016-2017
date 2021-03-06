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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import android.app.Activity;
import android.graphics.Color;
import android.view.View;
/**
 * Created by acandidato on 2/13/17.
 */
@Autonomous(name = "BlueCenterAuto", group = "Pushbot")
public class BlueCenterAuto extends LinearOpMode {
    HardwarePushbot robot       = new HardwarePushbot();   // Use a Pushbot's hardware
    public ElapsedTime runtime = new ElapsedTime();

    //ODS Setup
    static final double     WHITE_THRESHOLD = 0.5;
    static final double     APPROACH_SPEED  = 0.5;

    //Encoder Setup
    static final double     COUNTS_PER_MOTOR_REV    = 1120;
    static final double     DRIVE_GEAR_REDUCTION    = 1;
    static final double     WHEEL_DIAMETER_INCHES   = 4.0;
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    //Drive speed and turn speed
    static final double     DRIVE_SPEED     = 0.7;
    static final double     TURN_SPEED      = 0.5;

    //Gyro
    static final double     HEADING_THRESHOLD       = 1;
    static final double     P_TURN_COEFF            = 0.1;
    static final double     P_DRIVE_COEFF           = 0.15;
    int zAccumulated;
    int target = 0;

    //flyWheel
    double power = 0;
    static final double INCREMENT   = 0.01; // amount to ramp motor each CYCLE_MS cycle
    static final double DECREASE    = -0.01;
    static final double MAX_FWD     =  1.00;     // Maximum FWD power applied to motor
    static final double MAX_REV     =  0;
    boolean rampUp  = true;

    //Servo
    double          intakeOffset  = 0.0 ;                  // Servo mid position
    final double    INTAKE_SPEED  = 0.02 ;


    //gyro
    ModernRoboticsI2cGyro   gyro;                    // Additional Gyro device

    boolean LEDState = false;

    @Override
    public void runOpMode(){
        robot.init(hardwareMap);
        // Motor Setup
        robot.leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.flyWheelMotor.setPower(power);

        // get a reference to a Modern Robotics GyroSensor object.
        //Gyro int setup
        int xVal, yVal, zVal = 0;
        int heading = 0;
        int angleZ = 0;

        // reset Color Sensor
        robot.color.enableLed(false);

        // set beacon pushers to 0(0.16) before user presses play
        robot.leftBeacon.setPosition(0.16);
        robot.rightBeacon.setPosition(0.16);

        //Calibrate Gyro
        telemetry.addData(">", "Gyro Calibrating. Do Not Move!");
        telemetry.update();
        robot.gyro.calibrate();
        while (!isStopRequested() && robot.gyro.isCalibrating()){
            sleep(50);
            idle();
        }
        telemetry.addData(">", "Gyro Calibrated. Press Start.");
        telemetry.update();
        robot.color.enableLed(LEDState);

        waitForStart();
        gyroDrive(DRIVE_SPEED, 50, 0);
        shoot();
        gyroDrive(DRIVE_SPEED, 7, 0);
        gyroTurn(TURN_SPEED, 130);
        gyroDrive(DRIVE_SPEED, -45, 0);
        align(3000);
        decideColor();
        // pushBeacon(2000);
        // gyroDrive(DRIVE_SPEED, 5, 0);
        // gyroTurn(TURN_SPEED, -90);
        // gyroDrive(DRIVE_SPEED, 36, 180);
        // align(3000);
        // pushBeacon(2000);
    }
    
    public void shoot(){
        double power = 0;
        
        for(double power=0; power<1.0; power+=0.01){
            robot.flyWheelMotor.setPower(power);
        }
        
        power = 0;
        
        robot.limiter.setPosition(0.2);
        
        robot.limiter.setPosition(1.0);
        
        wait(2000);
        
        robot.limiter.setPosition(0.2);
        
        robot.limiter.setPosition(1.0);
    }
    
    public void pushBeacon(double time){
        while(runtime.seconds()<=time) {
            if (robot.color.red() < robot.color.blue()) {
                robot.leftBeacon.setPosition(0.3);
                gyroDrive(DRIVE_SPEED, 2, 0);
            }
            if (robot.color.red() > robot.color.blue()) {
                robot.rightBeacon.setPosition(0.3);
                gyroDrive(DRIVE_SPEED, 2, 0);
            }
        }
    }

    public void decideColor(){

        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F,0F,0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(R.id.RelativeLayout);

        // convert the RGB values to HSV values.
        Color.RGBToHSV(robot.color.red() * 8, robot.color.green() * 8, robot.color.blue() * 8, hsvValues);

        // send the info back to driver station using telemetry function.
        telemetry.addData("LED", false ? "On" : "Off");
        telemetry.addData("Clear", robot.color.alpha());
        telemetry.addData("Red  ", robot.color.red());
        telemetry.addData("Green", robot.color.green());
        telemetry.addData("Blue ", robot.color.blue());
        telemetry.addData("Hue", hsvValues[0]);

        // change the background color to match the color detected by the RGB sensor.
        // pass a reference to the hue, saturation, and value array as an argument
        // to the HSVToColor method.
        relativeLayout.post(new Runnable() {
            public void run() {
                relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
            }
        });

        if(isBlue() == true)
        {
            robot.leftBeacon.setPosition(1.0);
        }else{
            robot.rightBeacon.setPosition(1.0);
        }

        telemetry.update();

    }

    public boolean isBlue(){
        if(robot.color.blue() > robot.color.red() && robot.color.blue() > robot.color.green())
        {
            return true;
        }else{
            return false;
        }
    }

    public boolean isRed(){
        if(robot.color.red() > robot.color.blue() && robot.color.red() > robot.color.green())
        {
            return true;
        }else{
            return false;
        }
    }

    public void shoot(){
        while(power < 1.0)
        power += INCREMENT;
        if (power >= MAX_FWD) {
            power = MAX_FWD;
            rampUp = true;
            robot.limiter.setPosition(.2);
            sleep(500);
            robot.limiter.setPosition(1);
            sleep(500);
            robot.limiter.setPosition(.2);
            sleep(500);
            robot.limiter.setPosition(1);
            sleep(500);
        }
        power -= INCREMENT;
        if (power <= 0.0) {
            power = 0.0;
            rampUp = true;
        }
    }
    public void align(double time){
        while(runtime.seconds()<=time) {
            while ((robot.odsSensorI.getLightDetected() < 0.5) && (robot.odsSensorII.getLightDetected() < 0.5) && (robot.odsSensorIII.getLightDetected() < 0.5) && (robot.odsSensorIV.getLightDetected() < 0.5)) {
                robot.leftFrontMotor.setPower(-0.5);
                robot.leftBackMotor.setPower(-0.5);
                robot.rightBackMotor.setPower(0.0);
                robot.rightFrontMotor.setPower(0.0);
                sleep(700);
                robot.leftFrontMotor.setPower(0);
                robot.leftBackMotor.setPower(0);
                robot.rightBackMotor.setPower(-0.5);
                robot.rightFrontMotor.setPower(-0.5);
                sleep(700);
            }
            while (((robot.odsSensorI.getLightDetected() > 0.5) || (robot.odsSensorII.getLightDetected() > 0.5) || (robot.odsSensorIII.getLightDetected() > 0.5)) && (robot.odsSensorIV.getLightDetected() < 0.5)) {
                robot.leftFrontMotor.setPower(-0.2);
                robot.leftBackMotor.setPower(-0.2);
                robot.rightBackMotor.setPower(-0.2);
                robot.rightFrontMotor.setPower(-0.2);
            }
            if ((robot.odsSensorII.getLightDetected() < 0.5) && (robot.odsSensorIV.getLightDetected() > 0.5)) {
                robot.leftFrontMotor.setPower(0.15);
                robot.leftBackMotor.setPower(0.15);
                robot.rightBackMotor.setPower(-0.2);
                robot.rightFrontMotor.setPower(-0.2);
            }
        }
    }
    public void gyroDrive (double speed, double distance, double angle){
        int     newLeftTarget;
        int     newRightTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        if (opModeIsActive()) {
            //Determine target position
            moveCounts = (int)(distance * COUNTS_PER_INCH);
            newLeftTarget = robot.leftBackMotor.getCurrentPosition() + moveCounts;
            newRightTarget = robot.rightBackMotor.getCurrentPosition() + moveCounts;

            //Set Target and activate RUN_TO_POSITION
            robot.leftBackMotor.setTargetPosition(newLeftTarget);
            robot.leftFrontMotor.setTargetPosition(newLeftTarget);

            robot.rightBackMotor.setTargetPosition(newRightTarget);
            robot.rightFrontMotor.setTargetPosition(newRightTarget);

            robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //start motion
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            robot.leftFrontMotor.setPower(speed);
            robot.leftBackMotor.setPower(speed);
            robot.rightBackMotor.setPower(speed);
            robot.rightFrontMotor.setPower(speed);

            //Keep looping while both motors are active
            while (opModeIsActive() && (robot.leftFrontMotor.isBusy() && robot.leftBackMotor.isBusy()) && (robot.rightFrontMotor.isBusy() && robot.rightBackMotor.isBusy())){
                //adjust speed on account of error
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                //motor correction must be reversed for driving in reverse
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed - steer;

                //normalize speeds
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0){
                    leftSpeed /= max;
                    rightSpeed /= max;
                }
                robot.leftFrontMotor.setPower(leftSpeed);
                robot.leftBackMotor.setPower(leftSpeed);
                robot.rightFrontMotor.setPower(rightSpeed);
                robot.rightBackMotor.setPower(rightSpeed);

                //Display driver status
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:%7d",      newLeftTarget,  newRightTarget);
                telemetry.addData("Actual",  "%7d:%7d",      robot.leftBackMotor.getCurrentPosition(), robot.rightBackMotor.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                telemetry.update();
            }

            //Stop
            robot.leftFrontMotor.setPower(0);
            robot.leftBackMotor.setPower(0);
            robot.rightBackMotor.setPower(0);
            robot.rightFrontMotor.setPower(0);

            //Turn off RUN_TO_POSITION
            robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

    }
    public void gyroTurn (double speed, double angle){
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)){
            telemetry.update();
        }
    }
    public void gyroHold( double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();
        }

        // Stop all motion;
        robot.leftFrontMotor.setPower(0);
        robot.leftBackMotor.setPower(0);
        robot.rightFrontMotor.setPower(0);
        robot.rightBackMotor.setPower(0);
    }
    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        robot.leftFrontMotor.setPower(leftSpeed);
        robot.leftBackMotor.setPower(leftSpeed);
        robot.rightFrontMotor.setPower(rightSpeed);
        robot.rightBackMotor.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - robot.gyro.getIntegratedZValue();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

}
