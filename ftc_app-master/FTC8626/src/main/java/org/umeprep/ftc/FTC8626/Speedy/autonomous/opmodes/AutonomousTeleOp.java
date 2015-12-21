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

package org.umeprep.ftc.FTC8626.Speedy.autonomous.opmodes;

import org.swerverobotics.library.interfaces.TeleOp;
import org.umeprep.ftc.FTC8626.Speedy.autonomous.DriveMoveDirection;
import org.umeprep.ftc.FTC8626.Speedy.autonomous.DriveTurnDirection;

import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 * Linear Tele Op Mode for Autonomous movement
 * <p/>
 */
@TeleOp(name="AutonomousTeleOp", group="FTC8626")
public class AutonomousTeleOp extends LinearOpMode {

    private DcMotor motorRight;
    private DcMotor motorLeft;
    private TouchSensor v_sensor_touch;
    private OpticalDistanceSensor v_sensor_ods;

    public boolean v_warning_generated = false;
    public String v_warning_message;

    Double turnDuration = 1.085;

    @Override
    public void runOpMode() throws InterruptedException {
        initializeRobot();

        waitForStart();

        makeSomeMoves();
    }

    private void initializeRobot() {
        motorLeft = hardwareMap.dcMotor.get("motor_1");   // Get the name of the real motors
        motorRight = hardwareMap.dcMotor.get("motor_2");  // Get the name of the real motors
        motorLeft.setDirection(DcMotor.Direction.REVERSE);

        //
        // Connect the sensors.
        //
        try {
            v_sensor_touch = hardwareMap.touchSensor.get("Touch_1");
        } catch (Exception p_exeception) {
            m_warning_message("Touch_1");
            //DbgLog.msg (p_exeception.getLocalizedMessage ());

            v_sensor_touch = null;
        }

        try {
            v_sensor_ods = hardwareMap.opticalDistanceSensor.get("ODS_1");
        } catch (Exception p_exeception) {
            m_warning_message("ODS_1");
            //DbgLog.msg (p_exeception.getLocalizedMessage ());

            v_sensor_touch = null;

            /*
            try
            {
                v_sensor_ods = hardwareMap.opticalDistanceSensor.get
                        ( "sensor_eopd"
                        );
            }
            catch (Exception p_exeception_eopd)
            {
                try
                {
                    v_sensor_ods = hardwareMap.opticalDistanceSensor.get
                            ( "sensor_EOPD"
                            );
                }
                catch (Exception p_exeception_EOPD)
                {
                    m_warning_message ("sensor_ods/eopd/EOPD");
                    /*
                    DbgLog.msg
                            ( "Can't map sensor_ods nor sensor_eopd, nor" +
                                            " sensor_EOPD ("
                                            + p_exeception_EOPD.getLocalizedMessage ()
                                            + ").\n"
                            );
                    *

                    v_sensor_ods = null;
                }
            }
            */
        }

    } // init

    private void makeSomeMoves() throws InterruptedException {

//        telemetry.addData("makeSomeMoves", "Starting move forward");

        //    double duration = 1.2;
        //move(DriveMoveDirection.Forward, .3, duration);
        //      telemetry.addData("makeSomeMoves", "After move forward");

        //move(DriveMoveDirection.Backward, .3, duration);

        //telemetry.addData("makeSomeMoves3", "After move backward");

        //Thread.sleep(1000);

        //testTurn(0);
/*
        turn(DriveTurnDirection.Left, 45);
        move(DriveMoveDirection.Forward, .2, 1);

        turn(DriveTurnDirection.Right, 135);
        move(DriveMoveDirection.Forward, .5, .7);

        move(DriveMoveDirection.Backward, .2, 1.5);
        turn(DriveTurnDirection.Left, 90);
        move(DriveMoveDirection.Forward, .8, .5);

*/
        /*
        boolean floatIt = true;
        move(DriveMoveDirection.Forward, .8, 1, floatIt);
        */
    }

    private void move(DriveMoveDirection robotMoveDirection, double movePower, double durationInSeconds) throws InterruptedException {

        telemetry.addData("move1", "After set power");

        if (robotMoveDirection == DriveMoveDirection.Forward) {
            telemetry.addData("Direction", "equals Forward");
        } else {
            telemetry.addData("Direction", "equals Backward");
            movePower = -movePower;
        }

        setMotorPower(movePower);

        continueAction(durationInSeconds);

        telemetry.addData("move2", "Sleep 1/4 second");
        Thread.sleep(250);
    }

    private void continueAction(double durationInSeconds) throws InterruptedException {
        telemetry.addData("doAction1", "Duration secs: " + durationInSeconds);

        long durationInMilliseconds = (long) (durationInSeconds * 1000);
        Thread.sleep(durationInMilliseconds);   //Thread.Sleep may experience an error while running so it can "throw an exception"

        stopRobot();

        telemetry.addData("doAction2", "After stop");

        Thread.sleep(1000);
    }

    private void setMotorPower(double power) {
        setMotorPower(power, power);
    }

    private void setMotorPower(double rightPower, double leftPower) {
        //telemetry.addData("setMotorPower1", "Starting");
        //telemetry.addData("setMotorPower2", "LeftPower: " + leftPower); //motorLeft.getPower());
        //telemetry.addData("setMotorPower3", "Right power: " + rightPower); //motorRight.getPower());

        motorRight.setPower(rightPower);
        motorLeft.setPower(leftPower);
    }

    // Incorporate this if needed
    //update_telemetry (); // Update common telemetry
    //update_gamepad_telemetry ();

    private void stopRobot() {
        telemetry.addData("StopRobot1", "Preparing to stop");
        setMotorPower(0);
        telemetry.addData("StopRobot2", "After stopping");
    }

    /*
    private void move(DriveMoveDirection robotDirection, double movePower, double durationInSeconds, boolean floatStop) {

        setDirectedMotorPower(robotDirection, movePower);
        continueAction(durationInSeconds);

        if (floatStop) {
            floatRobot();
        }
    }
    */
/*
    private void turn(DriveTurnDirection direction, float moveAngle) throws InterruptedException {
        telemetry.addData("Turn", "Starting turn");
        telemetry.addData("Turn", "Move angle: " + moveAngle);

        float angle = Range.clip(moveAngle, 0, 180);
        telemetry.addData("Turn", "Angle after clip: " + angle);

        // The following code will be replaced by the gyro if we get it working
        float right = 0;
        float left = 0;

        // Translate the move Angle into an amount of right or left turn between 1 and -1
        if (direction == DriveTurnDirection.Left) {
            angle = -angle;
        }

        telemetry.addData("Turn", "Angle after direction: " + angle);

        right = 0 + angle / 180;
        left = 0 - angle / 180;

        telemetry.addData("Turn", "Right: " + right);
        telemetry.addData("Turn", "Left: " + left);

        // clip the right/left values so that the values never exceed +/- 1
        right = Range.clip(right, -1, 1);
        left = Range.clip(left, -1, 1);

        telemetry.addData("Turn", "Right after clip: " + right);
        telemetry.addData("Turn", "Left after clip: " + left);

        telemetry.addData("Turn", "Setting power start");

        // write the values to the motors
        setMotorPower(right, left);

        telemetry.addData("Turn", "After set power: " + left);

        continueAction(turnDuration);

        telemetry.addData("Turn", "After continue: " + left);
    }
*/
/*
    private void floatRobot() {
        telemetry.addData("Floating1", "Starting float");
        motorRight.setPowerFloat();
        motorLeft.setPowerFloat();
    }
*/
/*
    private void testTurn(double seed) throws InterruptedException {

        double first = seed + .2;
        double second = seed + .4;
        double third = seed + .6;
        double fourth = seed + .8;
        double fifth = seed + 1;

        turnSpecificTime(DriveTurnDirection.Right, 180, first);
        turnSpecificTime(DriveTurnDirection.Right, 180, second);
        turnSpecificTime(DriveTurnDirection.Right, 180, third);
        turnSpecificTime(DriveTurnDirection.Right, 180, fourth);
        turnSpecificTime(DriveTurnDirection.Right, 180, fifth);
    }
*/
    private void turn(DriveTurnDirection direction, float moveAngle) throws InterruptedException {
        turnSpecificTime(direction, moveAngle, turnDuration);
    }

    private void turnSpecificTime(DriveTurnDirection direction, float moveAngle, double duration) throws InterruptedException {

        double angle = Range.clip(moveAngle, 0, 180);

        // The following code will be replaced by the gyro if we get it working
        float right = 0;
        float left = 0;

        // Translate the move Angle into an amount of right or left turn between 1 and -1
        if (direction == DriveTurnDirection.Left) {
            moveAngle = -moveAngle;
        }

        right = 0 + moveAngle / 180;
        left = 0 - moveAngle / 180;

        // clip the right/left values so that the values never exceed +/- 1
        right = Range.clip(right, -1, 1);
        left = Range.clip(left, -1, 1);

        // write the values to the motors
        setMotorPower(right, left);
        continueAction(duration);
    }


    private void testTurn(double seed) throws InterruptedException {
        //double interval = 1;

        for (int diff = 1; diff <= 5; diff++) {
            double turnAmount = seed + turnDuration; // interval * diff;
            turnSpecificTime(DriveTurnDirection.Right, 180, turnAmount);
            telemetry.addData("testTurn", "Turn amount: " + turnAmount);
            Thread.sleep(1000);
        }
    }


    /*
    private void makeSomeMoves() throws InterruptedException
    {
        // write the values to the motors
        double testPower = .5;
        motorRight.setPower(testPower);
        motorLeft.setPower(testPower);

        telemetry.addData("Text", "AutonomousTeleOp");
        telemetry.addData("left power: ", motorLeft.getPower());
        telemetry.addData("right power: ", motorRight.getPower());

        int moveDurationInSeconds = 2;
        int moveDuration = moveDurationInSeconds * 1000;  //# of Seconds x 1000 -> Sleep needs milliseconds, 2 seconds = 2000 milliseconds
        Thread.sleep(moveDuration);   //Thread.Sleep may experience an error while running so it can "throw an exception"

        testPower = 0;
        motorRight.setPower(testPower);
        motorLeft.setPower(testPower);

        telemetry.addData("Text", "AutonomousTeleOp");
        telemetry.addData("left power: ", motorLeft.getPower());
        telemetry.addData("right power: ", motorRight.getPower());

    }
    */


    /**
     * Mutate the warning message by ADDING the specified message to the current
     * message; set the warning indicator to true.
     * <p/>
     * A comma will be added before the specified message if the message isn't
     * empty.
     */
    void m_warning_message(String p_exception_message)

    {
        if (v_warning_generated) {
            v_warning_message += ", ";
        }
        v_warning_generated = true;
        v_warning_message += p_exception_message;

    } // m_warning_message

}
