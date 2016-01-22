

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
import org.umeprep.ftc.FTC8626.Speedy.Utility;

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
    DcMotor motorHook;

    double driveMotorPowerFactor = .2;  // 0 to 1, higher number gives motors more power, use lower numbers for testing
    double hookMotorPowerFactor = 1;  // 0 to 1, higher number gives motors more power, use lower numbers for testing

    //DcMotor motorRightKick;
    //DcMotor motorLeftKick;

    ////anything with four slashes is part of the servo mechanics
    Servo tapeMeasureUpDown;
    private Servo servoClimberDumper;
    private Servo servoButtonPusher;


    // position of the arm servo.
    double HOOK_MIN_POSITION = 0;
    double HOOK_MAX_POSITION = .7;
    double HOOK_INITIAL_POSITION = 0;

    double tapeMeasureUpDownPosition = HOOK_INITIAL_POSITION;
    // amount to change the tape measure up down servo position by
    double tapeMeasureUpDownDelta = 0.0008;

    //double dumpClimbersPosition;


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
        servoClimberDumper = hardwareMap.servo.get("Climber Dumper");
        servoButtonPusher = hardwareMap.servo.get("Button Pusher");

        try {
            setServoPositions();
        }
        catch (InterruptedException ex)
        {}
        //dumpClimbers = hardwareMap.servo.get("servo_6");

        // assign the starting position of the servos

        //dumpClimbersPosition = 0.2;


        // write position values to the wrist and claw servo
        //dumpClimbers.setPosition(dumpClimbersPosition);
        telemetry.addData("Robot says","Hi");
    }

    private void setServoPositions() throws InterruptedException {

        tapeMeasureUpDown.setPosition(HOOK_INITIAL_POSITION);
        servoClimberDumper.setPosition(0);
        servoButtonPusher.setPosition(0);
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
		 */

        //*********
        // Drive with joysticks
        //*********
        // Currently testing using both game controllers
        float left = gamepad1.left_stick_y; // + gamepad2.left_stick_y;
        float right = gamepad1.right_stick_y; // + gamepad2.right_stick_y;

        //clip the right/left values so that the values never exceed +/- 1
        right = Range.clip(right, -1, 1);
        left = Range.clip(left, -1, 1);

        // scale the joystick value to make it easier to control
        // the robot more precisely at slower speeds.
        right = (float) Utility.scaleInput(right);
        left =  (float) Utility.scaleInput(left);

        // write the values to the motors
        motorRight.setPower(right);
        motorLeft.setPower(left);
        //
        //
        //

        //*********
        // Extend or retract the hook
        //*********
        double hookOutSpeed = .2;
        double hookInSpeed = -.2;

        //clip the hook speed values so that the values never exceed +/- 1
        hookOutSpeed = Range.clip(hookOutSpeed, -1, 1);
        hookInSpeed = Range.clip(hookInSpeed, -1, 1);

        //set the hook motor power
        if (gamepad1.dpad_up) {
            motorHook.setPower(hookOutSpeed * hookMotorPowerFactor);
        }
        else if (gamepad1.dpad_down) {
            motorHook.setPower(hookInSpeed * hookMotorPowerFactor);
        }
        else {
            motorHook.setPower(0);
        }
        /*
        if (gamepad2.dpad_up) {
            motorHook.setPower(hookOutSpeed);
        }
        if (gamepad2.dpad_down) {
            motorHook.setPower(hookInSpeed);
        }
        else {
            motorHook.setPower(0);
        }
        */
        //
        //
        //

        // ********
        // Change the height of the hook and tape measure
        // ********
        if (gamepad1.y) {
            tapeMeasureUpDownPosition += tapeMeasureUpDownDelta;
        }
        else if (gamepad1.b) {
            tapeMeasureUpDownPosition -= tapeMeasureUpDownDelta;
        }

        if (gamepad1.right_bumper && gamepad1.left_bumper){
            tapeMeasureUpDownPosition = HOOK_INITIAL_POSITION;
        }

        /*
        if (gamepad2.y) {
            tapeMeasureUpDownPosition += tapeMeasureUpDownDelta;
        }
        if (gamepad2.b) {
            tapeMeasureUpDownPosition -= tapeMeasureUpDownDelta;
        }
        if (gamepad2.a) {
            tapeMeasureUpDownPosition = .2;
        }
        */

        //clip the hook speed values so that the values never go below 0 or above .6
        tapeMeasureUpDownPosition = Range.clip(tapeMeasureUpDownPosition, HOOK_MIN_POSITION, HOOK_MAX_POSITION);
        tapeMeasureUpDown.setPosition(tapeMeasureUpDownPosition);
        //
        //
        //


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


        // Set servos

        // clip the position values so that they never exceed their allowed range.
        //tapeMeasureUpDownPosition = Range.clip(armPosition, ARM_MIN_RANGE, ARM_MAX_RANGE);
        //clawPosition = Range.clip(clawPosition, CLAW_MIN_RANGE, CLAW_MAX_RANGE);



		/*
		 * Send telemetry data back to driver station. Note that if we are using
		 * a legacy NXT-compatible motor controller, then the getPower() method
		 * will return a null value. The legacy NXT-compatible motor controllers
		 * are currently write only.
		 */
       // telemetry.addData("Text", "*** Robot Data***");
        telemetry.addData("left tgt pwr",  "left  pwr: " + String.format("%.2f", left));
        telemetry.addData("right tgt pwr", "right pwr: " + String.format("%.2f", right));
        telemetry.addData("Robot says","Long live the King.");
    }

    /*
     * Code to run when the op mode is first disabled goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
     */
    @Override
    public void stop() {

    }
}
