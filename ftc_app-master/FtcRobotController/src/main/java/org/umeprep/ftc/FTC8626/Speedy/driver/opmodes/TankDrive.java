
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
import org.umeprep.ftc.FTC8626.Speedy.Motion;
import org.umeprep.ftc.FTC8626.Speedy.Utility;
import org.umeprep.ftc.FTC8626.Speedy.autonomous.DriveMoveDirection;

/**
 * TankDrive Mode
 * <p>
 * Enables control of the robot via the gamepad
 */


//Blade is the best, Then Ian, but Andre is our Leader!

//@TeleOp(name="TankDrive", group="FTC8626")
public class TankDrive extends OpMode {

    // Wheel motors
    DcMotor wheelMotor1;
    DcMotor wheelMotor2;

    // Hook motor
    DcMotor motorHook;

    // Servos
    private Servo servoTapeMeasureUpDown;
    private Servo servoClimberDumperArm;
    private Servo servoDebrisPusher;
    private Servo servoZipLineLeft;
    private Servo servoZipLineRight;

    DriveMoveDirection robotDirection;
    public final double DRIVE_MOTOR_POWER_FACTOR = .7;  // 0 to 1, higher number gives motors more power, use lower numbers for testing

    double HOOK_MOTOR_POWER_FACTOR = 1;  // 0 to 1, higher number gives motors more power, use lower numbers for testing

    // Debris pusher constants
    private final double DEBRIS_PUSHER_UP = .70;

    private final double DEBRIS_PUSHER_DOWN = .22;

    // position of the arm servo.
    double HOOK_MIN_POSITION = 0;
    double HOOK_MAX_POSITION = .6;
    double HOOK_AX_POSITION = .37;   // Intended for initial mountain ascent

    private final double HOOK_OUT_SPEED = 1;
    private final double HOOK_IN_SPEED = -.8;

    private final double DEFAULT_DRIVE_MOTOR_POWER = .6;

    double tapeMeasureUpDownPosition = HOOK_MAX_POSITION;
    double tapeMeasureUpDownDelta = 0.001;   // amount to change the tape measure up down servo position

    private final double CLIMBER_DUMPER_ARM_IN = 1;
    private final double CLIMBER_DUMPER_ARM_OUT = 0;
    private final double CLIMBER_DUMPER_EXTEND_SLOW_CHANGE = 0.01;  // amount to change the climber dumper up down
    private final double CLIMBER_DUMPER_RETRACT_SLOW_CHANGE = 0.0005;  // amount to change the climber dumper up down
    //private final double CLIMBER_DUMPER_LID_CLOSED_POSITION = 0.15;
    //private final double CLIMBER_DUMPER_LID_OPEN_POSITION = 0.75;

    //    Below is the Climber ZipLine Release initial positions
    private double INIT_ZIP_LINE_LEFT_POSITION = 1;
    private double INIT_ZIP_LINE_RIGHT_POSITION = 0;

    double zipLineLeftUpDownPosition = INIT_ZIP_LINE_LEFT_POSITION;
    double zipLineRightUpDownPosition = INIT_ZIP_LINE_RIGHT_POSITION;

//    Below is the Climber ZipLine Release positions
    private final double ZIP_LINE_LEFT_UP_POSITION = 1;
    private final double ZIP_LINE_LEFT_DOWN_POSITION = 0.35;

    private final double ZIP_LINE_RIGHT_UP_POSITION = 0.15;
    private final double ZIP_LINE_RIGHT_DOWN_POSITION = .8;

    private Motion motion;

    /*
     * Code to run when the op mode is first enabled goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */
    @Override
    public void init()  {

        try {

            // Get wheel motors
            wheelMotor1 = hardwareMap.dcMotor.get("Wheel 1");
            wheelMotor2 = hardwareMap.dcMotor.get("Wheel 2");

            // Initialize wheel motors
            wheelMotor1.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
            wheelMotor2.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
            wheelMotor1.setDirection(DcMotor.Direction.FORWARD);
            wheelMotor2.setDirection(DcMotor.Direction.REVERSE);
            robotDirection = DriveMoveDirection.Forward;
            motion = new Motion(wheelMotor1, wheelMotor2);

            // Get hook motor
            motorHook = hardwareMap.dcMotor.get("Hook");
            motorHook.setDirection(DcMotor.Direction.REVERSE);

            // Get servos
            servoTapeMeasureUpDown = hardwareMap.servo.get("Hook Control");
            servoClimberDumperArm = hardwareMap.servo.get("Climber Dumper Arm");
            servoDebrisPusher = hardwareMap.servo.get("Debris Pusher");
            servoZipLineLeft = hardwareMap.servo.get("Zip Line Left");
            servoZipLineRight = hardwareMap.servo.get("Zip Line Right");

            setServoPositions();
        }
        catch (Exception ex)
        {
            telemetry.addData("error", ex.getMessage());
        }
    }

    private void setServoPositions() {

        // assign the starting position of the servos
        servoTapeMeasureUpDown.setPosition(HOOK_MAX_POSITION);
        servoClimberDumperArm.setPosition(CLIMBER_DUMPER_ARM_IN);
        servoZipLineLeft.setPosition(INIT_ZIP_LINE_LEFT_POSITION);
        servoZipLineRight.setPosition(INIT_ZIP_LINE_RIGHT_POSITION);
        initializeDebrisPusher();
    }

    @Override
    public void loop() {

        //*********
        // Drive with joysticks
        //*********
        if (gamepad1.dpad_left && gamepad1.a) {
            robotDirection = DriveMoveDirection.Backward;
            wheelMotor1.setDirection(DcMotor.Direction.REVERSE);
            wheelMotor2.setDirection(DcMotor.Direction.FORWARD);
        }
        else if (gamepad1.dpad_right && gamepad1.a) {
            robotDirection = DriveMoveDirection.Forward;
            wheelMotor1.setDirection(DcMotor.Direction.FORWARD);
            wheelMotor2.setDirection(DcMotor.Direction.REVERSE);
        }


        // Read power to motors from game controllers
        double leftStick = gamepad1.left_stick_y * DRIVE_MOTOR_POWER_FACTOR;
        double rightStick = gamepad1.right_stick_y * DRIVE_MOTOR_POWER_FACTOR;

        //clip the right/left values so that the values never exceed +/- 1
        rightStick = Range.clip(rightStick, -1, 1);
        leftStick = Range.clip(leftStick, -1, 1);

        // scale the joystick value to make it easier to control
        // the robot more precisely at slower speeds.
        rightStick = Utility.scaleInput(rightStick);
        leftStick =  Utility.scaleInput(leftStick);

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


        //*********
        // Extend or retract the hook
        //********

        double hookOutSpeed = HOOK_OUT_SPEED * HOOK_MOTOR_POWER_FACTOR;
        double hookInSpeed = HOOK_IN_SPEED * HOOK_MOTOR_POWER_FACTOR;

        //clip the hook speed values so that the values never exceed +/- 1
        hookOutSpeed = Range.clip(hookOutSpeed, -1, 1);
        hookInSpeed = Range.clip(hookInSpeed, -1, 1);

        //set the hook motor power
        if (gamepad1.dpad_up) {
            motorHook.setPower(hookOutSpeed);
            setDebrisPusher(DebrisPusherDirection.Up);
        }
        else if (gamepad1.dpad_down) {
            motorHook.setPower(hookInSpeed);
            setDebrisPusher(DebrisPusherDirection.Up);

            // Set the wheels to "float" so they spin freely when being pulled up the mountain (ramp)
            wheelMotor2.setPowerFloat();
            wheelMotor1.setPower(-.08);
        }
        else {
            motorHook.setPower(0);
        }



        // ********
        // Change the position of the debris pusher
        // ********
        if (gamepad1.left_trigger > 0 && gamepad1.a) {
            setDebrisPusher(DebrisPusherDirection.Up);
        }
        else if (gamepad1.right_trigger > 0 && gamepad1.a) {
            setDebrisPusher(DebrisPusherDirection.Down);
        }


        // ********
        // Change position of zip line triggers
        // ********

        if (gamepad1.back && gamepad1.right_trigger > 0) {
            servoZipLineLeft.setPosition(ZIP_LINE_LEFT_UP_POSITION);
        }
        else if (gamepad1.back && gamepad1.left_trigger > 0) {
            servoZipLineLeft.setPosition(ZIP_LINE_LEFT_DOWN_POSITION);
        }

        if (gamepad1.start && gamepad1.left_trigger > 0) {
            servoZipLineRight.setPosition(ZIP_LINE_RIGHT_UP_POSITION);
        }
        else if (gamepad1.start && gamepad1.right_trigger > 0) {
            servoZipLineRight.setPosition(ZIP_LINE_RIGHT_DOWN_POSITION);
        }


        // ********
        // Dump the climbers
        // ********

        if (gamepad1.back && gamepad1.y) {
            extendClimberDumperArm();
        }
        else if (gamepad1.start && gamepad1.y) {
            retractDumper();
        }

        // ********
        // Change the height of the hook and tape measure
        // ********
        else if (gamepad1.y) {
            tapeMeasureUpDownPosition += tapeMeasureUpDownDelta;
        }
        else if (gamepad1.b) {
            tapeMeasureUpDownPosition -= tapeMeasureUpDownDelta;
        }
        else if (gamepad1.x && gamepad1.a){
            tapeMeasureUpDownPosition = HOOK_AX_POSITION;
        }
        else if (gamepad1.x){
            tapeMeasureUpDownPosition = HOOK_MIN_POSITION;
        }


        // ********
        // Change the position of the debris pusher
        // ********
        if (gamepad1.left_trigger > 0 && gamepad1.right_trigger > 0) {
            motion.simpleMoveBackwards();
        }
        /*
        else if (gamepad1.left_trigger > 0 && gamepad1.right_bumper) {
            // motion.stopDriveMotors();
            motion.setMotorPower(0);
        }
        */

        //
        //clip the hook speed values so that the values never go below the limits of the hardware
        //
        tapeMeasureUpDownPosition = Range.clip(tapeMeasureUpDownPosition, HOOK_MIN_POSITION, HOOK_MAX_POSITION);
        servoTapeMeasureUpDown.setPosition(tapeMeasureUpDownPosition);

    }

    /*
     * Code to run when the op mode is first disabled goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
     */
    @Override
    public void stop() {
        setDebrisPusher(DebrisPusherDirection.Down);
    }

    private void initializeDebrisPusher() { //} throws InterruptedException {

        //initializeDebrisPusher to the down position
        servoDebrisPusher.setPosition(DEBRIS_PUSHER_DOWN);
    }

    private void setDebrisPusher(DebrisPusherDirection direction) {

        if (direction == DebrisPusherDirection.Up) {
            // Move the pusher up or down
            servoDebrisPusher.setPosition(DEBRIS_PUSHER_UP);
        } else {
            // Move the pusher up or down
            servoDebrisPusher.setPosition(DEBRIS_PUSHER_DOWN);
        }
    }

    private void retractDumper() {
        double armPosition = servoClimberDumperArm.getPosition();
        armPosition += CLIMBER_DUMPER_EXTEND_SLOW_CHANGE;
        armPosition = Range.clip(armPosition, CLIMBER_DUMPER_ARM_OUT, CLIMBER_DUMPER_ARM_IN);
        servoClimberDumperArm.setPosition(armPosition);
    }


    private void extendClimberDumperArm() {

        double armPosition = servoClimberDumperArm.getPosition();
        armPosition -= CLIMBER_DUMPER_RETRACT_SLOW_CHANGE;
        armPosition = Range.clip(armPosition, CLIMBER_DUMPER_ARM_OUT, CLIMBER_DUMPER_ARM_IN);
        servoClimberDumperArm.setPosition(armPosition);
    }

}
