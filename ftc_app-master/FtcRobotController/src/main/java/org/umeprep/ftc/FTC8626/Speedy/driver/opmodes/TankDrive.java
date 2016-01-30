

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
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.umeprep.ftc.FTC8626.Speedy.DebrisPusherDirection;
import org.umeprep.ftc.FTC8626.Speedy.Utility;
import org.umeprep.ftc.FTC8626.Speedy.autonomous.DriveMoveDirection;

/**
 * TankDrive Mode
 * <p>
 * Enables control of the robot via the gamepad
 */


//Ian is the best, Then Andrew, but Andre is our Leader!

//@TeleOp(name="TankDrive", group="FTC8626")
public class TankDrive extends OpMode {

    DcMotor wheelMotor1;
    DcMotor wheelMotor2;
    DriveMoveDirection robotDirection;
    DcMotor motorHook;

    private Servo servoDebrisPusherRight;
    private Servo servoDebrisPusherLeft;
    private Servo servoDebrisPusherMiddle;

    public final double DRIVE_MOTOR_POWER_FACTOR = .7;  // 0 to 1, higher number gives motors more power, use lower numbers for testing
    double hookMotorPowerFactor = 1;  // 0 to 1, higher number gives motors more power, use lower numbers for testing

    //DcMotor motorRightKick;
    //DcMotor motorLeftKick;

    ////anything with four slashes is part of the servo mechanics
    private Servo servoTapeMeasureUpDown;
    private Servo servoClimberDumper;
    private boolean dumperClicked = false;
    //private Servo servoButtonPusher;


    // position of the arm servo.
    double HOOK_MIN_POSITION = 0;
    double HOOK_MAX_POSITION = .7;
    double HOOK_INITIAL_POSITION = 0;

    double tapeMeasureUpDownPosition = HOOK_INITIAL_POSITION;
    // amount to change the tape measure up down servo position by
    double tapeMeasureUpDownDelta = 0.0008;

    double dumpClimbersPosition;


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
        wheelMotor1 = hardwareMap.dcMotor.get("Wheel 1");
        wheelMotor2 = hardwareMap.dcMotor.get("Wheel 2");

        wheelMotor1.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        wheelMotor2.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);

        wheelMotor1.setDirection(DcMotor.Direction.FORWARD);
        wheelMotor2.setDirection(DcMotor.Direction.REVERSE);
        robotDirection = DriveMoveDirection.Forward;

        motorHook = hardwareMap.dcMotor.get("Hook");
        motorHook.setDirection(DcMotor.Direction.REVERSE);

        // Servo code
        servoTapeMeasureUpDown = hardwareMap.servo.get("Hook Control");
        servoClimberDumper = hardwareMap.servo.get("Climber Dumper");
        //servoButtonPusher = hardwareMap.servo.get("Button Pusher");

        servoDebrisPusherRight = hardwareMap.servo.get("Debris Pusher Right");
        servoDebrisPusherLeft = hardwareMap.servo.get("Debris Pusher Left");
        servoDebrisPusherMiddle = hardwareMap.servo.get("Debris Pusher Middle");

        try {
            setServoPositions();
        }
        catch (InterruptedException ex)
        {}
        //dumpClimbers = hardwareMap.servo.get("servo_6");

        // assign the starting position of the servos

        dumpClimbersPosition = .1;


        // write position values to the wrist and claw servo
        //dumpClimbers.setPosition(dumpClimbersPosition);
    }

    private void setServoPositions() throws InterruptedException {

        servoTapeMeasureUpDown.setPosition(HOOK_INITIAL_POSITION);
        // servoClimberDumper.setPosition(0);
        //servoButtonPusher.setPosition(0);

        initializeDebrisPusher();
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
        double leftStick = gamepad1.left_stick_y; //* DRIVE_MOTOR_POWER_FACTOR;
        double rightStick = gamepad1.right_stick_y; //* DRIVE_MOTOR_POWER_FACTOR;

        //clip the right/left values so that the values never exceed +/- 1
        rightStick = Range.clip(rightStick, -1, 1);
        leftStick = Range.clip(leftStick, -1, 1);

        // scale the joystick value to make it easier to control
        // the robot more precisely at slower speeds.
        rightStick = Utility.scaleInput(rightStick);
        leftStick =  Utility.scaleInput(leftStick);

        if (gamepad2.left_bumper) {
            robotDirection = DriveMoveDirection.Backward;
            wheelMotor1.setDirection(DcMotor.Direction.REVERSE);
            wheelMotor2.setDirection(DcMotor.Direction.FORWARD);
        }
        else if (gamepad2.right_bumper) {
            robotDirection = DriveMoveDirection.Forward;
            wheelMotor1.setDirection(DcMotor.Direction.FORWARD);
            wheelMotor2.setDirection(DcMotor.Direction.REVERSE);
        }

        // set power to the motors based on which end of the robot is expected to be forward
        if (robotDirection == DriveMoveDirection.Forward) {
            wheelMotor1.setPower(rightStick);
            wheelMotor2.setPower(leftStick);
        } else {
            wheelMotor1.setPower(leftStick);
            wheelMotor2.setPower(rightStick);
        }

        // Display power telemetry to the driver
        telemetry.addData("left tgt pwr", "left  pwr: " + String.format("%.2f", leftStick));
        telemetry.addData("right tgt pwr", "right pwr: " + String.format("%.2f", rightStick));

        //
        //
        //

        //*********
        // Extend or retract the hook
        //********
        double hookOutSpeed = 1;
        double hookInSpeed = -.7;

        //clip the hook speed values so that the values never exceed +/- 1
        hookOutSpeed = Range.clip(hookOutSpeed, -1, 1);
        hookInSpeed = Range.clip(hookInSpeed, -1, 1);

        //set the hook motor power
        if (gamepad1.dpad_up) {
            motorHook.setPower(hookOutSpeed * hookMotorPowerFactor);
            try {
                setDebrisPusher(DebrisPusherDirection.Up,false);
            } catch (InterruptedException ex) {}
        }
        else if (gamepad1.dpad_down) {
            motorHook.setPower(hookInSpeed * hookMotorPowerFactor);
            try {
                setDebrisPusher(DebrisPusherDirection.Up,false);
            } catch (InterruptedException ex) {}

            // Set the wheels to "float" so they spin freely when going up the mountain (ramp)
            wheelMotor1.setPowerFloat();
            wheelMotor2.setPowerFloat();
        }
        else {
            motorHook.setPower(0);
        }

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

        if (gamepad1.x){
            tapeMeasureUpDownPosition = HOOK_INITIAL_POSITION;
        }

        //clip the hook speed values so that the values never go below 0 or above .6
        tapeMeasureUpDownPosition = Range.clip(tapeMeasureUpDownPosition, HOOK_MIN_POSITION, HOOK_MAX_POSITION);
        servoTapeMeasureUpDown.setPosition(tapeMeasureUpDownPosition);
        //
        //
        //

        // ********
        // Change the position of the debris pusher
        // ********
        try {
            if (gamepad1.left_trigger > 0) {
                setDebrisPusher(DebrisPusherDirection.Up, false);
            }
            else if (gamepad1.right_trigger > 0 && !gamepad1.dpad_up && !gamepad1.dpad_down) {
                setDebrisPusher(DebrisPusherDirection.Up, false);
                setDebrisPusher(DebrisPusherDirection.Down);
            }
        }
        catch (InterruptedException ex)
        {
            telemetry.addData("Error",ex.getMessage());
        }

        //
        //
        //

        // ********
        // Dump the climbers
        // ********
        if (gamepad1.right_bumper && gamepad1.a)
        {
            try {
                    dumpClimbers();
            }
            catch (InterruptedException ex)
            {
                telemetry.addData("Error",ex.getMessage());
            }
        }
        else if (gamepad1.left_bumper)
        {
            try {
                retractDumper();
            }
            catch (InterruptedException ex)
            {
                telemetry.addData("Error",ex.getMessage());
            }
        }



		/*
		 * Send telemetry data back to driver station. Note that if we are using
		 * a legacy NXT-compatible motor controller, then the getPower() method
		 * will return a null value. The legacy NXT-compatible motor controllers
		 * are currently write only.
		 */
        // telemetry.addData("Text", "*** Robot Data***");
        //telemetry.addData("left tgt pwr",  "left  pwr: " + String.format("%.2f", leftStick));
        //telemetry.addData("right tgt pwr", "right pwr: " + String.format("%.2f", rightStick));
    }

    /*
     * Code to run when the op mode is first disabled goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
     */
    @Override
    public void stop() {

    }

    private void initializeDebrisPusher() throws InterruptedException {

        // Move the pusher up
        servoDebrisPusherRight.setPosition(.15);
        servoDebrisPusherLeft.setPosition(.9);
        Thread.sleep(500);

        // Set the middle brace before moving the pusher down
        double pusherMiddlePosition = .45;
        servoDebrisPusherMiddle.setPosition(pusherMiddlePosition);
        Thread.sleep(300);

        // Move the pusher down
        servoDebrisPusherRight.setPosition(.9);
        servoDebrisPusherLeft.setPosition(.15);
    }

    private void setDebrisPusher(DebrisPusherDirection direction) throws InterruptedException {
        setDebrisPusher(direction, true);
    }

    private void setDebrisPusher(DebrisPusherDirection direction, boolean bracePusher) throws InterruptedException {

        double pusherMiddlePosition = .45;

        if (direction == DebrisPusherDirection.Up) {
            // Move the pusher up
            servoDebrisPusherRight.setPosition(.15);
            servoDebrisPusherLeft.setPosition(.9);

            // Move the pusher up before moving the middle brace out of the way
            Thread.sleep(500);
            servoDebrisPusherMiddle.setPosition(0);

        } else {

            if (bracePusher) {
                // Set the middle brace before moving the pusher down
                servoDebrisPusherMiddle.setPosition(pusherMiddlePosition);
                Thread.sleep(500);
            } else {
                servoDebrisPusherMiddle.setPosition(0);
            }

            // Move the pusher up or down
            servoDebrisPusherRight.setPosition(.9);
            servoDebrisPusherLeft.setPosition(.15);
        }
    }

    private void dumpClimbers() throws InterruptedException {

        dumpClimbersPosition += .005;
        dumpClimbersPosition = Range.clip(dumpClimbersPosition,.1,.8);
        servoClimberDumper.setPosition(dumpClimbersPosition);

    }

    private void retractDumper() throws InterruptedException {

        dumpClimbersPosition -= .005;
        dumpClimbersPosition = Range.clip(dumpClimbersPosition,0,.8);
        servoClimberDumper.setPosition(dumpClimbersPosition);
    }
}
