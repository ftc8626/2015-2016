

/* Copyright (c) 2014 Qualcomm Technologies Inc

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

package org.umeprep.ftc.FTC8626.Speedy.driver.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.swerverobotics.library.interfaces.TeleOp;

/**
 * TankDrive Mode
 * <p>
 * Enables control of the robot via the gamepad
 */


//Ian is the best, Then Andrew, but Andre is our Leader!

@TeleOp(name="TankDrive", group="FTC8626")
public class TankDrive extends OpMode {

    DcMotor motorRight;
    DcMotor motorLeft;
    //DcMotor motorBack;
    DcMotor motorHook;

    //DcMotor motorRightKick;
    //DcMotor motorLeftKick;

    ////anything with four slashes is part of the servo mechanics
    Servo tapeMeasureUpDown;
    ////Servo dumpClimbers;

    // position of the arm servo.
    double tapeMeasureUpDownPosition;

    //double dumpClimbersPosition;

    // amount to change the tape measure up down servo position by
    double tapeMeasureUpDownDelta = 0.1;

    /**
     * Constructor
     */
    public TankDrive() {

    }

    /*
     * Code to run when the op mode is first enabled goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */
    @Override
    public void init()  {


		/*
		 * Use the hardwareMap to get the dc motors and servos by name. Note
		 * that the names of the devices must match the names used when you
		 * configured your robot and created the configuration file.
		 */

		/*
		 * For the demo Tetrix K9 bot we assume the following,
		 *   There are two motors "motor_1" and "motor_2"
		 *   "motor_1" is on the right side of the bot.
		 *   "motor_2" is on the left side of the bot and reversed.
		 */
        //motorRightKick = hardwareMap.dcMotor.get("motor_4");
        //motorLeftKick = hardwareMap.dcMotor.get("motor_3");
        motorLeft = hardwareMap.dcMotor.get("Wheel 1");
        motorRight = hardwareMap.dcMotor.get("Wheel 2");
        //motorBack = hardwareMap.dcMotor.get("motor_3");
        motorLeft.setDirection(DcMotor.Direction.REVERSE);

        motorHook = hardwareMap.dcMotor.get("Hook");
        motorHook.setDirection(DcMotor.Direction.REVERSE);

        // Servo code
        tapeMeasureUpDown = hardwareMap.servo.get("Hook Control");

        try {
            setIncline();
        }
        catch (InterruptedException ex)
        {}
        //dumpClimbers = hardwareMap.servo.get("servo_6");

        // assign the starting position of the servos

        //dumpClimbersPosition = 0.2;


        // write position values to the wrist and claw servo
        //dumpClimbers.setPosition(dumpClimbersPosition);
    }

    private void setIncline() throws InterruptedException {

        tapeMeasureUpDownPosition = .5;
    }

    /*
     * This method will be called repeatedly in a loop
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
     */
    @Override
    public void loop() {

		/*
		 * Gamepad 1
		 *
		 * Gamepad 1 controls the motors via the left stick, and it controls the
		 * wrist/claw via the a,b, x, y buttons
		 */

        // throttle: left_stick_y ranges from -1 to 1, where -1 is full up, and
        // 1 is full down
        // direction: left_stick_x ranges from -1 to 1, where -1 is full left
        // and 1 is full right
        float direction2 = gamepad1.right_stick_y;
        float direction = gamepad1.left_stick_y;
        float left = direction;
        float right = direction2;

        // Extend or retract the hook
        if (gamepad1.dpad_up && !gamepad1.dpad_down) {
            motorHook.setPower(.2);
        }
        else if (gamepad1.dpad_down && !gamepad1.dpad_up) {
            motorHook.setPower(-.2);
        }
        else {
            motorHook.setPower(0);
        }

        // Change the height of the hook and tape measure
        if (gamepad1.y) {
            tapeMeasureUpDown.setPosition(.4);
        }
        else if (gamepad1.b) {
            tapeMeasureUpDown.setPosition(.5);
        }

        ////if (gamepad1.a) {
        // if the A button is pushed on gamepad1, increment the position of
        // the arm servo.
        // motorRightKick.setPower(.8);
        // motorLeftKick.setPower(-.8);
        ////}
        ////else {
        // motorRightKick.setPower(0);
        // motorLeftKick.setPower(0);
        ////}

        //clip the right/left values so that the values never exceed +/- 1
        ////right = Range.clip(right, -1, 1);
        ////left = Range.clip(left, -1, 1);

        // scale the joystick value to make it easier to control
        // the robot more precisely at slower speeds.
        ////right = (float)scaleInput(right);
        ////left =  (float)scaleInput(left);

        // write the values to the motors
        motorRight.setPower(right * .3);
        motorLeft.setPower(left * .3);
        //motorRightKick.setPower(right);
        //motorLeftKick.setPower(left);

        // Set servos
        // update the position of the claw
        ////if (gamepad1.dpad_up) {
        //tapeMeasureUpDownPosition += tapeMeasureUpDownDelta;
        ////tapeMeasureUpDownPosition = .7;
        ////}

        /*
        if (gamepad1.dpad_down) {

            //tapeMeasureUpDownPosition -= tapeMeasureUpDownDelta;
            ////tapeMeasureUpDownPosition = 1;
        }
        */

        //tapeMeasureUpDownPosition = .7;

        ////tapeMeasureUpDown.setPosition(tapeMeasureUpDownPosition);


        // clip the position values so that they never exceed their allowed range.
        //tapeMeasureUpDownPosition = Range.clip(armPosition, ARM_MIN_RANGE, ARM_MAX_RANGE);
        //clawPosition = Range.clip(clawPosition, CLAW_MIN_RANGE, CLAW_MAX_RANGE);



		/*
		 * Send telemetry data back to driver station. Note that if we are using
		 * a legacy NXT-compatible motor controller, then the getPower() method
		 * will return a null value. The legacy NXT-compatible motor controllers
		 * are currently write only.
		 */
        telemetry.addData("Text", "*** Robot Data***");
        telemetry.addData("left tgt pwr",  "left  pwr: " + String.format("%.2f", left));
        telemetry.addData("right tgt pwr", "right pwr: " + String.format("%.2f", right));
    }

    /*
     * Code to run when the op mode is first disabled goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
     */
    @Override
    public void stop() {

    }


    /*
     * This method scales the joystick input so for low joystick values, the
     * scaled value is less than linear.  This is to make it easier to drive
     * the robot more precisely at slower speeds.
     */
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
