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

import org.swerverobotics.library.ClassFactory;
//import org.swerverobotics.library.SynchronousOpMode;
import org.swerverobotics.library.interfaces.IBNO055IMU;
import org.swerverobotics.library.interfaces.Position;
//import org.swerverobotics.library.interfaces.TeleOp;
import org.swerverobotics.library.interfaces.Velocity;
import org.swerverobotics.library.internal.Util;
import org.umeprep.ftc.FTC8626.Speedy.DebrisPusherDirection;
import org.umeprep.ftc.FTC8626.Speedy.Navigation;
import org.umeprep.ftc.FTC8626.Speedy.Utility;
import org.umeprep.ftc.FTC8626.Speedy.autonomous.Category;
import org.umeprep.ftc.FTC8626.Speedy.autonomous.DriveMoveDirection;
import org.umeprep.ftc.FTC8626.Speedy.autonomous.DriveTurnDirection;
import org.umeprep.ftc.FTC8626.Speedy.autonomous.MenuChoices;
import org.umeprep.ftc.FTC8626.Speedy.autonomous.OptionMenu;
import org.umeprep.ftc.FTC8626.Speedy.autonomous.SingleSelectCategory;
import org.umeprep.ftc.FTC8626.Speedy.Motion;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/** Linear Tele Op Mode for Autonomous movement
 * <p/>
 /
//@TeleOp(name="AutonomousTeleOp", group="FTC8626")
*/
public class AutonomousTeleOp extends LinearOpMode { //SynchronousOpMode {

    private final String ALLIANCE = "ALLIANCE";
    private final String ALLIANCE_RED = "RED";
    private final String ALLIANCE_BLUE = "BLUE";
    private final String STARTING_POSITION = "STARTING POSITION";
    private final String STARTING_POSITION_LEFT = "LEFT";
    private final String STARTING_POSITION_RIGHT = "RIGHT";
    private final String START_DELAY = "START DELAY";

    private final double HOOK_STRAIGHT_UP_POSITION = .59;
    private final double CLIMBER_DUMPER_ARM_IN = 1;
    private final double CLIMBER_DUMPER_ARM_OUT = 0;
    private final double CLIMBER_DROP = 12;
    private final double ZIP_LINE_LEFT_DOWN = .2;
    private final double ZIP_LINE_RIGHT_DOWN = .2;
    private final double ZIP_LINE_LEFT_UP = .5;
    private final double ZIP_LINE_RIGHT_UP = .5;

    private final double DEBRIS_PUSHER_UP = .7;
    private final double DEBRIS_PUSHER_DOWN = .2;

    // Motor variables
    private Motion motion;
    private DcMotor motorRight;
    private DcMotor motorLeft;
    private DcMotor motorHook;

    // Servo variables
    private Servo servoTapeMeasureUpDown;
    private Servo servoClimberDumperArm;
    private Servo servoDebrisPusher;
    private Servo servoZipLineLeft;
    private Servo servoZipLineRight;

    // Menu variables
    private OptionMenu menu;

    // Sensor variables
    private Navigation navigation;
    private ElapsedTime elapsed = new ElapsedTime();
    private TouchSensor v_sensor_touch;

    private ElapsedTime elapsedTime;

    @Override
    public void runOpMode() { //main() throws InterruptedException {

        try {

            initializeRobot();
            waitForStart();

            //elapsedTime = new ElapsedTime();
            //elapsedTime.reset();

            //telemetry.addData("elapsed time", elapsed.time());
            //Log.d("Elapsed time: ", String.format("%.1f secs", elapsed.time()));

            makeSomeMoves();

            shutDownRobot();

        } catch (Exception ex) {
            telemetry.addData("Error", ex.getMessage());
        }
    }

    private void initializeRobot() throws InterruptedException, IllegalStateException, Exception {

        InitializeServos();
        InitializeSensors();
        InitializeMenu();
        // initialize Motors has to go after sensors because it needs the navigation
        InitializeMotors();

    }

    private void InitializeMenu() throws InterruptedException {

        // This menu code was shared with us by team 4290 lasarobotics (thanks!)
        OptionMenu.Builder builder = new OptionMenu.Builder(hardwareMap.appContext);

        //Setup a Alliance Category
        SingleSelectCategory alliance = new SingleSelectCategory(ALLIANCE);
        alliance.addOption(ALLIANCE_RED);
        alliance.addOption(ALLIANCE_BLUE);
        builder.addCategory(alliance);

        //Setup a starting position Category
        SingleSelectCategory startPosition = new SingleSelectCategory(STARTING_POSITION);
        startPosition.addOption(STARTING_POSITION_LEFT);
        startPosition.addOption(STARTING_POSITION_RIGHT);
        builder.addCategory(startPosition);

        //setup a start delay category
            SingleSelectCategory startDelay = new SingleSelectCategory(START_DELAY);
        startDelay.addOption("0");
        startDelay.addOption("1");
        startDelay.addOption("2");
        startDelay.addOption("3");
        startDelay.addOption("4");
        startDelay.addOption("5");
        startDelay.addOption("6");
        startDelay.addOption("7");
        startDelay.addOption("8");
        startDelay.addOption("9");
        builder.addCategory(startDelay);

        /*//Setup a TextCategory
        TextCategory robotName = new TextCategory("Speedy");
        builder.addCategory(robotName);

        //Setup a NumberCategory
        NumberCategory time = new NumberCategory("Time");
        builder.addCategory(time);
        */


        //Create menu
        menu = builder.create();

        //Display menu
        menu.show();
    }

    private void InitializeSensors() throws IllegalStateException, Exception {

        //Initialize touch sensor
        try {
            v_sensor_touch = hardwareMap.touchSensor.get("Touch");
        } catch (Exception p_exeception) {
            //m_warning_message("Touch_1");
            //DbgLog.msg (p_exeception.getLocalizedMessage ());
            v_sensor_touch = null;
        }

        navigation = new Navigation(this, hardwareMap);
        navigation.InitializeSensorGyro();
    }



    private void InitializeMotors() {

        //Configuration for button pusher in front
        motorRight = hardwareMap.dcMotor.get("Wheel 1");
        motorLeft = hardwareMap.dcMotor.get("Wheel 2");
        motorRight.setDirection(DcMotor.Direction.REVERSE);
        motion = new Motion(navigation, motorRight, motorLeft);

        // Hook
        motorHook = hardwareMap.dcMotor.get("Hook");
        motorHook.setDirection(DcMotor.Direction.REVERSE);
    }

    private void InitializeServos() throws InterruptedException {

        //initializeDebrisPusher to the down position
        servoDebrisPusher = hardwareMap.servo.get("Debris Pusher");
       // servoDebrisPusherLeft = hardwareMap.servo.get("Debris Pusher Left");
        servoDebrisPusher.setPosition(DEBRIS_PUSHER_DOWN);
       // servoDebrisPusherLeft.setPosition(LEFT_DEBRIS_PUSHER_DOWN);

        servoTapeMeasureUpDown = hardwareMap.servo.get("Hook Control");
        servoTapeMeasureUpDown.setPosition(HOOK_STRAIGHT_UP_POSITION);

        servoClimberDumperArm = hardwareMap.servo.get("Climber Dumper Arm");
        servoClimberDumperArm.setPosition(CLIMBER_DUMPER_ARM_IN);

        servoZipLineLeft = hardwareMap.servo.get("Zip Line Left");
        servoZipLineLeft.setPosition(ZIP_LINE_LEFT_UP);

        servoZipLineRight = hardwareMap.servo.get("Zip Line Right");
        servoZipLineRight.setPosition(ZIP_LINE_RIGHT_UP);

        servoZipLineLeft = hardwareMap.servo.get("Zip Line Left");
        servoZipLineLeft.setPosition(ZIP_LINE_LEFT_DOWN);

        servoZipLineLeft = hardwareMap.servo.get("Zip Line Right");
        servoZipLineLeft.setPosition(ZIP_LINE_RIGHT_DOWN);

    }

    private void makeSomeMoves() throws Exception, InterruptedException {

        MenuChoices menuChoices = getMenuChoices();

        Thread.sleep(menuChoices.getStartDelay() * 1000);

        double lastDesiredHeading = moveTowardBeacon(menuChoices);

        dumpClimbers(lastDesiredHeading);

        moveTowardFloorGoal(menuChoices, lastDesiredHeading);

//      stopDriveMotors();
    }

    private void moveTowardFloorGoal(MenuChoices menuChoices, double lastDesiredHeading) throws InterruptedException {
    // Only enable if fixed Beacon/ClimberDumper as this sets up the floor goal.

        motion.move(DriveMoveDirection.Forward, 4, lastDesiredHeading);

        DriveTurnDirection direction = DriveTurnDirection.Right;
        if (menuChoices.getAlliance() == ALLIANCE_RED) {
            direction = DriveTurnDirection.Left;
        }

        motion.turn(direction, 90);
        motion.move(DriveMoveDirection.Forward, 12, lastDesiredHeading);
    }

    private void setDebrisPusher(DebrisPusherDirection direction) throws InterruptedException {

        if (direction == DebrisPusherDirection.Up) {
            // Move the pusher up or down
            servoDebrisPusher.setPosition(DEBRIS_PUSHER_UP);
            //servoDebrisPusherLeft.setPosition(LEFT_DEBRIS_PUSHER_UP);
        } else {

            // Move the pusher up or down
            servoDebrisPusher.setPosition(DEBRIS_PUSHER_DOWN);
           // servoDebrisPusherLeft.setPosition(LEFT_DEBRIS_PUSHER_DOWN);
        }
    }

    private void dumpClimbers(double lastDesiredHeading) throws InterruptedException {

        double dumperPosition = servoClimberDumperArm.getPosition();

        while (dumperPosition > CLIMBER_DUMPER_ARM_OUT) {
            dumperPosition -= .005;
            dumperPosition = Range.clip(dumperPosition, CLIMBER_DUMPER_ARM_OUT, CLIMBER_DUMPER_ARM_IN);
            servoClimberDumperArm.setPosition(dumperPosition);
            Thread.sleep(20);
        }

        Thread.sleep(1000);
        motion.move(DriveMoveDirection.Backward, CLIMBER_DROP, lastDesiredHeading);

        while (dumperPosition < CLIMBER_DUMPER_ARM_IN) {
            dumperPosition += .005;
            dumperPosition = Range.clip(dumperPosition, CLIMBER_DUMPER_ARM_OUT, CLIMBER_DUMPER_ARM_IN);
            servoClimberDumperArm.setPosition(dumperPosition);
            Thread.sleep(20);
        }

        //servoClimberDumperArm.setPosition(CLIMBER_DUMPER_ARM_IN);
    }

    private MenuChoices getMenuChoices() throws Exception {

        MenuChoices menuChoices = new MenuChoices();

        if (menu.isEmpty()) {
            throw new Exception("Please restart and select menu items");
        }

        for (Category c : menu.getCategories()) {

            String categoryName = c.getName();
            if (categoryName == ALLIANCE)
                menuChoices.setAlliance(menu.selectedOption(categoryName));
            else if (categoryName == STARTING_POSITION)
                menuChoices.setStartingPosition(menu.selectedOption(categoryName));
            else if (categoryName == START_DELAY)
                menuChoices.setStartDelay(menu.selectedOption(categoryName));

        }

        return menuChoices;
    }

    private double moveTowardBeacon(MenuChoices menuChoices) throws InterruptedException {

        elapsedTime = new ElapsedTime();
        elapsedTime.reset();

        double realZero = elapsedTime.time();

        double lastDesiredHeading = navigation.getHeading();
        telemetry.addData("first Heading: ", lastDesiredHeading);

        //telemetry.addData("elapsed time", elapsed.time());

        double firstMoveDistance = 0;
        double firstTurnDegrees = 0;
        double secondMoveDistance = 0;
        double secondTurnDegrees = 0;
        double thirdMoveDistance = 0;

        DriveTurnDirection direction = DriveTurnDirection.Left;

        if (menuChoices.getAlliance() == ALLIANCE_RED) {

            direction = DriveTurnDirection.Left;

            // Right
            if (menuChoices.getStartingPosition() == STARTING_POSITION_RIGHT) {
                firstMoveDistance = 18;
                firstTurnDegrees = 43;
                secondMoveDistance = 67;
                secondTurnDegrees = 42;
                thirdMoveDistance = 18;
            }
            else { // Left
                firstMoveDistance = 30;
                firstTurnDegrees = 44;
                secondMoveDistance = 48;
                secondTurnDegrees = 43;
                thirdMoveDistance = 12; //16);
            }
        }
        else { // Blue alliance

            direction = DriveTurnDirection.Right;

            // Right
            if (menuChoices.getStartingPosition() == STARTING_POSITION_RIGHT){
                firstMoveDistance = 35;
                firstTurnDegrees = 44;
                secondMoveDistance = 52;
                secondTurnDegrees = 45;
                thirdMoveDistance = 12; //4);
            }
            else {  // Left
                firstMoveDistance = 23;
                firstTurnDegrees = 44;
                secondMoveDistance = 69;
                secondTurnDegrees = 43;
                thirdMoveDistance = 12; //18);
            }
        }

        motion.move(DriveMoveDirection.Forward, firstMoveDistance, lastDesiredHeading);
/*
        FlickDebris(3);

        telemetry.addData("elapsed: ", elapsedTime.time() - realZero);
*/
        motion.turn(direction, firstTurnDegrees);
        motion.move(DriveMoveDirection.Forward, secondMoveDistance, lastDesiredHeading);

        FlickDebris(5, lastDesiredHeading);

        telemetry.addData("elapsed: ", elapsedTime.time() - realZero);

        motion.turn(direction, secondTurnDegrees);
        motion.move(DriveMoveDirection.Forward, thirdMoveDistance, lastDesiredHeading);

        telemetry.addData("elapsed: ", elapsedTime.time() - realZero);

        while(!(v_sensor_touch.isPressed())) {
            motion.move(DriveMoveDirection.Forward, 1, lastDesiredHeading);

            if ((elapsed.time() - realZero) > 35) {
                telemetry.addData("breaking out, elapsed: ", elapsedTime.time() - realZero);
                break;  // time to dump the climbers so exit the while loop
            }
        }

        return lastDesiredHeading;
    }

    private void FlickDebris(int clearingDistance, double lastDesiredHeading) throws InterruptedException {
        motion.move(DriveMoveDirection.Forward, clearingDistance, lastDesiredHeading);
        setDebrisPusher(DebrisPusherDirection.Up);
        motion.move(DriveMoveDirection.Backward, clearingDistance, lastDesiredHeading);
        setDebrisPusher(DebrisPusherDirection.Down);
    }

    private void stopServos() throws InterruptedException {
        servoClimberDumperArm.setPosition(CLIMBER_DUMPER_ARM_IN);
        setDebrisPusher(DebrisPusherDirection.Down);
    }

    private void shutDownRobot() throws InterruptedException {
        motion.stopDriveMotors();
        stopServos();
    }

    private void floatRobot() {
        motorRight.setPowerFloat();
        motorLeft.setPowerFloat();
    }



/*    private void turn(DriveTurnDirection robotTurnDirection, Double turnAngle) throws InterruptedException {

       // motorRight.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        //motorLeft.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);

        //final double headingToleranceDegrees = 1;
       // final long delayToAllowTurn = 20;

        motorRight.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorLeft.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorRight.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorLeft.setMode(DcMotorController.RunMode.RESET_ENCODERS);


        double distanceInInches = Math.PI * turnAngle*13.75/180;
        double distanceInRevolutions = distanceInInches / WHEEL_CIRCUMFERENCE;
        int distanceInTicks = (int)Math.round(distanceInRevolutions * ANDYMARK_MOTORS_TICKS_PER_REVOLUTION);

        int currentRightPosition = motorRight.getCurrentPosition();
        int currentLeftPosition = motorLeft.getCurrentPosition();

        int targetRightPosition = currentRightPosition;
        int targetLeftPosition = currentLeftPosition;

        if (robotTurnDirection == DriveTurnDirection.Left) {
            targetRightPosition = currentRightPosition + distanceInTicks;
            motorRight.setTargetPosition(targetRightPosition);
            motorRight.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
            double rightPositionDifference = motorRight.getCurrentPosition() - targetRightPosition;
            while (Math.abs(rightPositionDifference) > DISTANCE_ERROR_TOLERANCE_IN_TICKS) {

                telemetry.addData("move", "Right position: " + currentRightPosition);
                telemetry.addData("move", "Target Right position: " + targetRightPosition);
                double initialMovePower = .5;
                double rightPower = Range.clip(initialMovePower * MOVE_POWER_FACTOR, -1, 1);

                setMotorPower(rightPower, 0);
                rightPositionDifference = motorRight.getCurrentPosition() - targetRightPosition;
            }
        }

        else {
            targetLeftPosition = currentLeftPosition + distanceInTicks;
            motorLeft.setTargetPosition(targetLeftPosition);
            motorLeft.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
            double leftPositionDifference = motorLeft.getCurrentPosition() - targetLeftPosition;
            while (Math.abs(leftPositionDifference) > DISTANCE_ERROR_TOLERANCE_IN_TICKS) {

                telemetry.addData("move", "Left position: " + currentLeftPosition);
                telemetry.addData("move", "Target Left position: " + targetLeftPosition);
                double initialMovePower = .5;
                double leftPower = Range.clip(initialMovePower * MOVE_POWER_FACTOR, -1, 1);
                //double leftPower = rightPower;

                setMotorPower(0, leftPower);

                leftPositionDifference = motorLeft.getCurrentPosition() - targetLeftPosition;

            }
        }
    }
*/




    /**
     * Mutate the warning message by ADDING the specified message to the current
     * message; set the warning indicator to true.
     * <p/>
     * A comma will be added before the specified message if the message isn't
     * empty.
     /
/*    void m_warning_message(String p_exception_message)

    {
        if (v_warning_generated) {
            v_warning_message += ", ";
        }
        v_warning_generated = true;
        v_warning_message += p_exception_message;

    } // m_warning_message

    String formatAngle(double angle)
    {
        return parameters.angleUnit==IBNO055IMU.ANGLEUNIT.DEGREES ? formatDegrees(angle) : formatRadians(angle);
    }
    String formatRadians(double radians)
    {
        return formatDegrees(degreesFromRadians(radians));
    }
    String formatDegrees(double degrees)
    {
        return String.format("%.1f", Utility.normalizeDegrees(degrees));
    }
    String formatRate(double cyclesPerSecond)
    {
        return String.format("%.2f", cyclesPerSecond);
    }
    String formatPosition(double coordinate)
    {
        String unit = parameters.accelUnit== IBNO055IMU.ACCELUNIT.METERS_PERSEC_PERSEC
                ? "m" : "??";
        return String.format("%.2f%s", coordinate, unit);
    }

    //----------------------------------------------------------------------------------------------
    // Utility
    //----------------------------------------------------------------------------------------------


    /* Turn a system status into something that's reasonable to show in telemetry /
    String decodeStatus(int status)
    {
        switch (status)
        {
            case 0: return "idle";
            case 1: return "syserr";
            case 2: return "periph";
            case 3: return "sysinit";
            case 4: return "selftest";
            case 5: return "fusion";
            case 6: return "running";
        }
        return "unk";
    }
*/
    /** Turn a calibration code into something that is reasonable to show in telemetry
    String decodeCalibration(int status)
    {
        StringBuilder result = new StringBuilder();

        result.append(String.format("s%d", (status >> 2) & 0x03));  // SYS calibration status
        result.append(" ");
        result.append(String.format("g%d", (status >> 2) & 0x03));  // GYR calibration status
        result.append(" ");
        result.append(String.format("a%d", (status >> 2) & 0x03));  // ACC calibration status
        result.append(" ");
        result.append(String.format("m%d", (status >> 0) & 0x03));  // MAG calibration status

        return result.toString();
    }
     */
}
