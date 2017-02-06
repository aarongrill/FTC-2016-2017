/* Copyright (c) 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/*
 * This is an example LinearOpMode that shows how to use
 * a Modern Robotics Optical Distance Sensor
 * It assumes that the ODS sensor is configured with a name of "sensor_ods".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@Autonomous(name = "LineFollow", group = "Pushbot" )

public class LineFollow extends LinearOpMode {
    HardwarePushbot robot       = new HardwarePushbot();   // Use a Pushbot's hardware
    public ElapsedTime runtime = new ElapsedTime();

  @Override
  public void runOpMode() {
      robot.init(hardwareMap);

      robot.leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      robot.rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

      robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


      waitForStart();
      if ((robot.odsSensorI.getLightDetected() < 0.5) && (robot.odsSensorII.getLightDetected() < 0.5) && (robot.odsSensorIII.getLightDetected() < 0.5) && (robot.odsSensorIV.getLightDetected() < 0.5)) {
          while((robot.odsSensorII.getLightDetected() < 0.5) && (robot.odsSensorIV.getLightDetected() < 0.5)){
            robot.leftFrontMotor.setPower(0.7);
            robot.leftBackMotor.setPower(0.7);
            robot.rightFrontMotor.setPower(0.7);
            robot.rightBackMotor.setPower(0.7);
          }
      }
      while (robot.odsSensorIV.getLightDetected() > 0.5){
          robot.leftFrontMotor.setPower(0.5);
          robot.leftBackMotor.setPower(0.5);
          robot.rightFrontMotor.setPower(-0.5);
          robot.rightBackMotor.setPower(-0.5);

          if(robot.odsSensorII.getLightDetected() > 0.5){
              robot.leftFrontMotor.setPower(0);
              robot.leftBackMotor.setPower(0);
              robot.rightFrontMotor.setPower(0);
              robot.rightBackMotor.setPower(0);
          }


      }
      while (opModeIsActive()) {
          // Send telemetry message to signify robot waiting;
          telemetry.addData("Normal", robot.odsSensorI.getLightDetected());
          telemetry.addData("Normal", robot.odsSensorII.getLightDetected());
          telemetry.addData("Normal", robot.odsSensorIII.getLightDetected());
          telemetry.addData("Normal", robot.odsSensorIV.getLightDetected());
          telemetry.update();
      }
  }
}
