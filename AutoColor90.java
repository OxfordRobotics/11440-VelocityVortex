/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN N O EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */


@Autonomous(name="AutoThrow90", group="Linear Opmode")  // @Autonomous(...) is the other common choice

public class AutoColor90 extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    private Hardware11440 robot = new Hardware11440();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        robot.init(hardwareMap);
        robot.frontCS.setI2cAddress(I2cAddr.create7bit(0x1e));
        robot.fbtmCS.setI2cAddress(I2cAddr.create7bit(0x26));
        robot.bbtmCS.setI2cAddress(I2cAddr.create7bit(0x2e));
        /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */


        //sick comment m9
        // eg: Set the drive motor directions
        // "Reverse" the motor that runs backwards when connected directly to the battery
        //robot.leftMotor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        //robot.rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        // run until the end of the match (driver presses STOP)

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();
        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while(opModeIsActive()) {

            // Drive forward 1 foot to shooting position
            robot.leftMotor.setPower(-.3);
            robot.rightMotor.setPower(-.3);

            while (Math.abs(robot.leftMotor.getCurrentPosition()) < 713 && Math.abs(robot.rightMotor.getCurrentPosition()) < 713) {

                telemetry.addData("LeftMotor", robot.leftMotor.getCurrentPosition());
                telemetry.addData("RightMotor", robot.rightMotor.getCurrentPosition());
                telemetry.update();
            }

            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);

            //shoot 2 debris

            robot.flyRight.setPower(1);
            robot.flyLeft.setPower(-1);
            sleep(200);
            robot.kickMotor.setPower(-0.22);
            sleep(750);
            robot.kickMotor.setPower(0.22);
            sleep(750);
            robot.kickMotor.setPower(-0.22);
            sleep(750);
            robot.kickMotor.setPower(0.22);
            sleep(750);
            robot.flyLeft.setPower(0);
            robot.flyRight.setPower(0);
            robot.kickMotor.setPower(0);

            // Drive forward 1 foot to clear corner vortex on turn
            robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            robot.leftMotor.setPower(-.3);
            robot.rightMotor.setPower(-.3);
            while (Math.abs(robot.leftMotor.getCurrentPosition()) < 713 && Math.abs(robot.rightMotor.getCurrentPosition()) < 713) {
                telemetry.addData("LeftMotor", robot.leftMotor.getCurrentPosition());
                telemetry.addData("RightMotor", robot.rightMotor.getCurrentPosition());
                telemetry.update();
            }

            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);

            //90 degree right turn toward beacon wall

            robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            robot.leftMotor.setPower(-0.2);
            robot.rightMotor.setPower(0.2);

            while (Math.abs(robot.leftMotor.getCurrentPosition()) < 550) {

            }

            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);

            //Drive forward towards beacon wall 4 feet

            robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            robot.leftMotor.setPower(-0.3);
            robot.rightMotor.setPower(-0.3);

            while (Math.abs(robot.leftMotor.getCurrentPosition()) < 2496 && Math.abs(robot.rightMotor.getCurrentPosition()) < 2496) {

            }

            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);

            //Left 90 degree turn towards while line

            robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            robot.leftMotor.setPower(0.2);
            robot.rightMotor.setPower(-0.2);


            while (Math.abs(robot.leftMotor.getCurrentPosition()) < 550) {

            }

            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);

            //Drive forward until front bottom color sensor detects white

            robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            robot.leftMotor.setPower(-0.15);
            robot.rightMotor.setPower(-0.15);

            while (robot.fbtmCS.alpha() < 12) {
                telemetry.addData("fbtmCS Alpha", robot.fbtmCS.alpha());
                telemetry.update();
            }

            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);

            //Right 90 degree turn towards beacon

            robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            robot.leftMotor.setPower(-0.15);
            robot.rightMotor.setPower(0.15);

            while (Math.abs(robot.leftMotor.getCurrentPosition()) < 550) {

            }

            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);

            //Drive forward towards beacon until close to beacon

            robot.leftMotor.setPower(-0.2);
            robot.rightMotor.setPower(-0.2);

            while (robot.ods.getRawLightDetected() < 0.2) {

            }

            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);

            break;

            }
        }

}


