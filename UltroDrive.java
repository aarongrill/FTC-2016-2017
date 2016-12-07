package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.HardwarePushbot;

/**
 * This file provides basic Telop driving for a Pushbot robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 *
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Ultro Drive", group="Pushbot")
public class UltroDrive extends OpMode{

    /* Declare OpMode members. */
    org.firstinspires.ftc.teamcode.HardwarePushbot robot = new HardwarePushbot(); // use the class created to define a Pushbot's hardware
    // could also use HardwarePushbotMatrix class.
    double          leftOffset  = 0.0 ;
    double          rightOffset = 0.0;
    // Servo mid position
    final double    SERVO_SPEED  = 0.02 ;                 // sets rate to move servo
    double distance;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        updateTelemetry(telemetry);
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
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        float left = gamepad1.left_stick_y;
        float right = -gamepad1.right_stick_y;
        //zipties intake
        //float intakeA = -gamepad2.left_stick_y;
        //elevator intake
        float intakeB = -gamepad2.right_stick_y;

        if (gamepad2.a){
            robot.flyWheelLeftMotor.setPower(-0.2);
            robot.flyWheelRightMotor.setPower(0.2);
        }
        if (gamepad2.b){
            robot.flyWheelLeftMotor.setPower(0);
            robot.flyWheelRightMotor.setPower(0);
        }
        robot.leftMotor.setPower(left);
        robot.rightMotor.setPower(right);
        //robot.intakeAMotor.setPower(intakeA);
        robot.intakeBMotor.setPower(intakeB);

        if(gamepad2.dpad_up)
        {
            robot.intakeAMotor.setPower(1.0);
        }
        if(gamepad2.dpad_down)
        {
            robot.intakeAMotor.setPower(-1.0);
        }
        if(gamepad2.dpad_right)
        {
            robot.intakeAMotor.setPower(0.0);
        }

        // Use gamepad left & right Bumpers to open and close the c=law
        if (gamepad1.right_bumper){
            robot.rightBeacon.setPosition(1.0);
        }
        if (gamepad1.left_bumper){
            robot.leftBeacon.setPosition(0.0);
        }
        if (gamepad1.right_trigger>0.0){
            robot.rightBeacon.setPosition(0.5);
        }
        if (gamepad1.left_trigger>0.0){
            robot.leftBeacon.setPosition(0.5);
        }
        // if (gamepad1.right_bumper)
        //     servoOffset += SERVO_SPEED;
        // else if (gamepad1.left_bumper)
        //     servoOffset -= SERVO_SPEED;

        // Move both servos to new position.  Assume servos are mirror image of each other.
        /*leftOffset = Range.clip(leftOffset, -0.5, 0.5);
        rightOffset = Range.clip(rightOffset, -0.5, 0.5);
        robot.leftBeacon.setPosition(robot.MID_SERVO + leftOffset);
        robot.rightBeacon.setPosition(robot.MID_SERVO - rightOffset);*/


        // Send telemetry message to signify robot running;
        telemetry.addData("leftB",  "Offset = %.2f", leftOffset);
        telemetry.addData("rightB", "Offset = %.2f", rightOffset);
        telemetry.addData("left", "left pwr " + String.format("%.2f", left));
        telemetry.addData("right", "right pwr " + String.format("%.2f", right));
        //   telemetry.addData("flyWheelLeft tgt pwr", "flyWheelLeft pwr " + String.format("%.2f", flyWheelLeft));
        //    telemetry.addData("flyWheelRight tgt pwr", "flyWheelRight pwr " + String.format("%.2f", flyWheelRight));
        updateTelemetry(telemetry);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
    double scaleInput(double dVal)  {
        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);

        // index should be positive.
        if (index < 0) {
            index = -index;
        }

        // index cannot exceed size of array minus 1.
        if (index > 16) {
            index = 16;
        }

        // get value from the array.
        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        // return scaled value.
        return dScale;
    }
}
