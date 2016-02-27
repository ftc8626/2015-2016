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
import org.umeprep.ftc.FTC8626.Speedy.DebrisPusherDirection;
import org.umeprep.ftc.FTC8626.Speedy.autonomous.Category;
import org.umeprep.ftc.FTC8626.Speedy.autonomous.DriveMoveDirection;
import org.umeprep.ftc.FTC8626.Speedy.autonomous.DriveTurnDirection;
import org.umeprep.ftc.FTC8626.Speedy.autonomous.MenuChoices;
import org.umeprep.ftc.FTC8626.Speedy.autonomous.OptionMenu;
import org.umeprep.ftc.FTC8626.Speedy.autonomous.SingleSelectCategory;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorController;
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

    private final int ANDYMARK_MOTORS_TICKS_PER_REVOLUTION = 1120;
    private final int DISTANCE_ERROR_TOLERANCE_IN_TICKS = 50;
    private final int WHEEL_DIAMETER = 4;
    private final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

    private final double HEADING_TOLERANCE_DEGREES = 1.1;

    // Adjust the MOVE_POWER_FACTOR based on the smoothness of floor surface
    private final double DRIVE_SLOW_POWER = .15;
    private final double DEFAULT_DRIVE_MOTOR_POWER = .6;
    private final double MOVE_POWER_FACTOR = .8;   // Multiplier to reduce the actual power
    private final double SLOW_MOVE_POWER_FACTOR = .2;
    private final double MOVE_POWER_HEADING_ADJUSTMENT = .008;

    private final double TURN_POWER = .15;
    private final double SLOW_TURN_POWER = .1;
    private final double SUPER_SLOW_TURN_POWER = .8;
    private final double TURN_POWER_FACTOR = 1.5;

    private final String ALLIANCE = "ALLIANCE";
    private final String ALLIANCE_RED = "RED";
    private final String ALLIANCE_BLUE = "BLUE";
    private final String STARTING_POSITION = "STARTING POSITION";
    private final String STARTING_POSITION_LEFT = "LEFT";
    private final String STARTING_POSITION_RIGHT = "RIGHT";
    private final String START_DELAY = "START DELAY";

    private final double HOOK_STRAIGHT_UP_POSITION = .59;
    private final double CLIMBER_DUMPER_ARM_MIN_POSITION = .0;
    private final double CLIMBER_DUMPER_ARM_MAX_POSITION = 1;
    private final double CLIMBER_DUMPER_LID_CLOSED_POSITION = 0.15;
    private final double CLIMBER_DUMPER_LID_OPEN_POSITION = 0.75;

    private final double DEBRIS_PUSHER_UP = .85;
    //private final double LEFT_DEBRIS_PUSHER_UP = .9;
    private final double DEBRIS_PUSHER_DOWN = .15;
  //  private final double LEFT_DEBRIS_PUSHER_DOWN = .2;

    // Motor variables
    private DcMotor motorRight;
    private DcMotor motorLeft;
    private DcMotor motorHook;

    // Servo variables
    private Servo servoTapeMeasureUpDown;
    private Servo servoClimberDumperArm;
    private Servo servoClimberDumperLid;
    private Servo servoDebrisPusher;
    //private Servo servoDebrisPusherLeft;

    // Menu variables
    private OptionMenu menu;

    // Sensor variables
    private IBNO055IMU v_sensor_gyro;
    private ElapsedTime elapsed = new ElapsedTime();
    private IBNO055IMU.Parameters parameters = new IBNO055IMU.Parameters();
    private TouchSensor v_sensor_touch;

    // Turn variables
    private double lastDesiredHeading;

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

        InitializeSensorGyro();
    }

    private void InitializeSensorGyro() throws IllegalStateException, Exception {

        parameters.angleUnit      = IBNO055IMU.ANGLEUNIT.DEGREES;
        parameters.accelUnit      = IBNO055IMU.ACCELUNIT.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        parameters.loggingTag     = "Gyro";

        I2cDevice device = hardwareMap.i2cDevice.get("I2cDevice");

        try {
            v_sensor_gyro = ClassFactory.createAdaFruitBNO055IMU(this, device, parameters);
            v_sensor_gyro.startAccelerationIntegration(new Position(), new Velocity());

            telemetry.addData("Robot says", "IMU acceleration started");

        } catch (Exception ex) {
            telemetry.addData("Error creating IMU Device", ex.getMessage());
            throw ex;
        }

        boolean isGyroCalibrated = v_sensor_gyro.isGyroCalibrated();

        for (int x=1; x < 15; x++) {
            if (!isGyroCalibrated) {
                Thread.sleep(1000);
                isGyroCalibrated = v_sensor_gyro.isGyroCalibrated();
            }
            else {
                telemetry.addData("IMU", "gyro calibrated");
                break;
            }
        }

        if (!isGyroCalibrated) {
            throw new Exception("Gyro failed to calibrate");
        }
    }

    private void InitializeMotors() {

        //Configuration for button pusher in front
        motorRight = hardwareMap.dcMotor.get("Wheel 1");
        motorLeft = hardwareMap.dcMotor.get("Wheel 2");
        motorRight.setDirection(DcMotor.Direction.REVERSE);

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
        servoClimberDumperArm.setPosition(CLIMBER_DUMPER_ARM_MIN_POSITION);

        servoClimberDumperLid = hardwareMap.servo.get("Climber Dumper Lid");
        servoClimberDumperLid.setPosition(CLIMBER_DUMPER_LID_CLOSED_POSITION);
    }

    private void makeSomeMoves() throws Exception, InterruptedException {

        MenuChoices menuChoices = getMenuChoices();
// this is to atempt a wit function for our allience
        Thread.sleep(menuChoices.getStartDelay()*1000);
        moveTowardBeacon(menuChoices);

        dumpClimbers();

//      moveTowardFloorGoal(menuChoices);

//      stopDriveMotors();
    }

/*    private void moveTowardFloorGoal(MenuChoices menuChoices) throws InterruptedException {
// Only enable if fixed Beacon/ClimberDumper as this sets up the floor goal.
        if (menuChoices.getAlliance() == ALLIANCE_RED) {
            turn(DriveTurnDirection.Left,78d);
            move(DriveMoveDirection.Forward, .3, 10);
        }
        else {
            turn(DriveTurnDirection.Right, 78d);
            move(DriveMoveDirection.Forward, .3, 10);
        }
    }
*/
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

    private void dumpClimbers() throws InterruptedException {

        double dumperPosition = servoClimberDumperArm.getPosition();

        while (dumperPosition < CLIMBER_DUMPER_ARM_MAX_POSITION) {
            dumperPosition += .005;
            dumperPosition = Range.clip(dumperPosition, CLIMBER_DUMPER_ARM_MIN_POSITION, CLIMBER_DUMPER_ARM_MAX_POSITION);
            servoClimberDumperArm.setPosition(dumperPosition);
            Thread.sleep(20);
        }

        Thread.sleep(1000);
        servoClimberDumperLid.setPosition(CLIMBER_DUMPER_LID_OPEN_POSITION);
        Thread.sleep(1000);

        while (dumperPosition > CLIMBER_DUMPER_ARM_MIN_POSITION) {
            dumperPosition -= .005;
            dumperPosition = Range.clip(dumperPosition, CLIMBER_DUMPER_ARM_MIN_POSITION, CLIMBER_DUMPER_ARM_MAX_POSITION);
            servoClimberDumperArm.setPosition(dumperPosition);
            Thread.sleep(20);
        }

        // Close lid and rest dumper arm on shelf
        servoClimberDumperLid.setPosition(CLIMBER_DUMPER_LID_CLOSED_POSITION);
        servoClimberDumperArm.setPosition(CLIMBER_DUMPER_ARM_MIN_POSITION);
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

    private void moveTowardBeacon(MenuChoices menuChoices) throws InterruptedException {

        elapsedTime = new ElapsedTime();
        elapsedTime.reset();

        double realZero = elapsedTime.time();

        lastDesiredHeading = getHeading();
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
            if (menuChoices.getStartingPosition() == STARTING_POSITION_RIGHT){
                firstMoveDistance = 18;
                firstTurnDegrees = 43;
                secondMoveDistance = 67;
                secondTurnDegrees = 42;
                thirdMoveDistance = 22; //18);
            }
            else { // Left
                firstMoveDistance = 30;
                firstTurnDegrees = 44;
                secondMoveDistance = 48;
                secondTurnDegrees = 43;
                thirdMoveDistance = 14; //16);
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
                thirdMoveDistance = 20; //18);
            }
        }

        move(DriveMoveDirection.Forward, firstMoveDistance);
/*
        FlickDebris(3);

        telemetry.addData("elapsed: ", elapsedTime.time() - realZero);
*/
        turn(direction, firstTurnDegrees);
        move(DriveMoveDirection.Forward, secondMoveDistance);

        FlickDebris(5);

        telemetry.addData("elapsed: ", elapsedTime.time() - realZero);

        turn(direction, secondTurnDegrees);
        move(DriveMoveDirection.Forward, thirdMoveDistance);

        telemetry.addData("elapsed: ", elapsedTime.time() - realZero);

        while(!(v_sensor_touch.isPressed())) {
            move(DriveMoveDirection.Forward, 1);

            if ((elapsed.time() - realZero) > 29) {
                telemetry.addData("breaking out, elapsed: ", elapsedTime.time() - realZero);
                break;  // time to dump the climbers so exit the while loop
            }
        }
    }

    private void FlickDebris(int clearingDistance) throws InterruptedException {
        move(DriveMoveDirection.Forward, clearingDistance);
        setDebrisPusher(DebrisPusherDirection.Up);
        move(DriveMoveDirection.Backward, clearingDistance);
        setDebrisPusher(DebrisPusherDirection.Down);
    }

    private void move(DriveMoveDirection robotMoveDirection, double distanceInInches) throws InterruptedException {
        double defaultMotorPower = DEFAULT_DRIVE_MOTOR_POWER;
        move(robotMoveDirection, defaultMotorPower, distanceInInches);
    }

    private void move(DriveMoveDirection robotMoveDirection, double movePower, double distanceInInches) throws InterruptedException {

        try {

            double initialHeading = lastDesiredHeading; //getHeading();
            double currentHeading = initialHeading;

            motorRight.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
            motorLeft.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);

            motorRight.setMode(DcMotorController.RunMode.RESET_ENCODERS);
            motorLeft.setMode(DcMotorController.RunMode.RESET_ENCODERS);

            double distanceInRevolutions = distanceInInches / WHEEL_CIRCUMFERENCE;
            int distanceInTicks = (int)Math.round(distanceInRevolutions * ANDYMARK_MOTORS_TICKS_PER_REVOLUTION);

            int currentRightPosition = motorRight.getCurrentPosition();
            int currentLeftPosition = motorLeft.getCurrentPosition();

            int targetRightPosition;
            int targetLeftPosition;
            if (robotMoveDirection == DriveMoveDirection.Forward) {
                targetRightPosition = currentRightPosition + distanceInTicks;
                targetLeftPosition = currentLeftPosition + distanceInTicks;
            }
            else {
                targetRightPosition = currentRightPosition - distanceInTicks;
                targetLeftPosition = currentLeftPosition - distanceInTicks;
            }

            motorRight.setTargetPosition(targetRightPosition);
            motorLeft.setTargetPosition(targetLeftPosition);

            motorRight.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
            motorLeft.setMode(DcMotorController.RunMode.RUN_TO_POSITION);

           // Start slow
            double initialMovePower = movePower;

            if (distanceInInches > 12) {
                // Adjust the power based on the factor (change based on smoothness of floor surface)

                initialMovePower = Range.clip(movePower * MOVE_POWER_FACTOR, -1, 1);

                for (int counter = 1; counter < 25; counter++) {
                    setMotorPower(initialMovePower/25 * counter);
                    Thread.sleep(50);
                }
            }
            else
            {
                // Adjust the power based on the factor (change based on smoothness of floor surface)
                initialMovePower = Range.clip(movePower * SLOW_MOVE_POWER_FACTOR, -1, 1);
            }

            double rightPositionDifference = motorRight.getCurrentPosition() - targetRightPosition;
            double leftPositionDifference = motorLeft.getCurrentPosition() - targetLeftPosition;

            while (Math.abs(rightPositionDifference) > DISTANCE_ERROR_TOLERANCE_IN_TICKS ||
                    Math.abs(leftPositionDifference) > DISTANCE_ERROR_TOLERANCE_IN_TICKS)  {

                double headingDifference = getHeadingDifference(lastDesiredHeading); //initialHeading);

                double rightPower = Range.clip(initialMovePower * MOVE_POWER_FACTOR, -1, 1);
                double leftPower = rightPower;

                if (headingDifference > 0) {
                    leftPower -= MOVE_POWER_HEADING_ADJUSTMENT;
                    rightPower += MOVE_POWER_HEADING_ADJUSTMENT;
                } else if (headingDifference < 0) {
                    leftPower += MOVE_POWER_HEADING_ADJUSTMENT;
                    rightPower -= MOVE_POWER_HEADING_ADJUSTMENT;
                }

                if (rightPositionDifference > DISTANCE_ERROR_TOLERANCE_IN_TICKS)
                    rightPower = -rightPower;
                if (leftPositionDifference > DISTANCE_ERROR_TOLERANCE_IN_TICKS)
                    leftPower = -leftPower;

                setMotorPower(rightPower, leftPower);

                Thread.sleep(20);

                rightPositionDifference = motorRight.getCurrentPosition() - targetRightPosition;
                leftPositionDifference = motorLeft.getCurrentPosition() - targetLeftPosition;
            }
        } catch (Exception ex) {
            telemetry.addData("Move", ex.getMessage());
        }

        stopDriveMotors();
        Thread.sleep(100);
    }

    private void turn(DriveTurnDirection direction, double turnAngle) throws InterruptedException {

        try {
            motorRight.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
            motorLeft.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);

            turnAngle = Range.clip(turnAngle, 0, 180);
            turnAngle = (direction == DriveTurnDirection.Left) ? -turnAngle : turnAngle;

            // Adjust the turnPowerFactor based on the smoothness of floor surface
            double turnPower = Range.clip(TURN_POWER * TURN_POWER_FACTOR, -1, 1);

            // Determine how far off the robot is currently
            //telemetry.addData("lastDesired","heading: " + lastDesiredHeading);
            double headingDrift = getAngleCorrectionAfterDrift(direction, lastDesiredHeading);
            //telemetry.addData("drift", "in heading: " + headingDrift);

            // Correct turn amount for existing drift
            if (Math.abs(headingDrift) > HEADING_TOLERANCE_DEGREES) {
                if (direction == DriveTurnDirection.Right)
                    turnAngle = normalizeDegrees(turnAngle + headingDrift);  // Drift will be a negative number in some cases
                else
                    turnAngle = normalizeDegrees(turnAngle - headingDrift);  // Drift will be a negative number in some cases
            }

            double desiredHeading = normalizeDegrees(lastDesiredHeading + turnAngle); //currentHeading + turnAngle);
            //telemetry.addData("desired","heading: " + desiredHeading);

            // Reset the last desired heading with the new direction
            lastDesiredHeading = desiredHeading;

            double headingDifference = getHeadingDifference(desiredHeading);
            while (Math.abs(headingDifference) > HEADING_TOLERANCE_DEGREES) {

                if (Math.abs(headingDifference) < 80) {
                    turnPower = SLOW_TURN_POWER * TURN_POWER_FACTOR;
                } else if (Math.abs(headingDifference) < 40) {
                    turnPower = SUPER_SLOW_TURN_POWER * TURN_POWER_FACTOR;
                }

                if (headingDifference > 0) { //direction == DriveTurnDirection.Left) {
                    // Turn left
                    motorRight.setPower(turnPower);
                    motorLeft.setPower(0);
                } else {
                    // Turn right
                    motorRight.setPower(0);
                    motorLeft.setPower(turnPower);
                }
/*
                double desiredMinusCurrent = desiredHeading - getHeading();

                if (desiredMinusCurrent > 180 || (desiredMinusCurrent < 0 && desiredMinusCurrent > -180)) {
                    // Turning left
                    motorRight.setPower(turnPower);
                    motorLeft.setPower(0);
                } else if ((desiredMinusCurrent > 0 && desiredMinusCurrent <= 180) || desiredMinusCurrent <= -180) {
                    // Turning right
                    motorRight.setPower(0);
                    motorLeft.setPower(turnPower);
                } else {
                    stopDriveMotors();
                }
*/

                headingDifference = getHeadingDifference(desiredHeading);
                Thread.sleep(20); // simulate the slower looping of a TeleOp mode - 50 times per second
            }

        } catch (Exception ex) {
            telemetry.addData("Error creating IMU: ", ex.getMessage());
        }

        stopDriveMotors();
        Thread.sleep(300);  // Reset the motors after turning
    }


    private void setMotorPower(double power) {
        setMotorPower(power, power);
    }

    private void setMotorPower(double rightPower, double leftPower) {
        motorRight.setPower(rightPower);
        motorLeft.setPower(leftPower);
    }

    private void stopDriveMotors() {
        setMotorPower(0);
    }

    private void stopServos() throws InterruptedException {
        servoClimberDumperArm.setPosition(CLIMBER_DUMPER_ARM_MIN_POSITION);
        servoClimberDumperLid.setPosition(CLIMBER_DUMPER_LID_CLOSED_POSITION);
        setDebrisPusher(DebrisPusherDirection.Down);
    }

    private void shutDownRobot() throws InterruptedException {
        stopDriveMotors();
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

    private double getAngleCorrectionAfterDrift(DriveTurnDirection direction, double desiredHeading) {

        double headingDifference = getHeadingDifference(desiredHeading);

        if (direction == DriveTurnDirection.Left)
            return headingDifference;
        else
            return -headingDifference;
    }


    private double getHeadingDifference(double desiredHeading) {
        double currentHeading = getHeading();

        double diff = currentHeading - desiredHeading;
        double absDiff = Math.abs(diff);

        if (absDiff > 180) {
            if (desiredHeading < currentHeading)
                diff = currentHeading - (360 + desiredHeading);
            else
                diff = (360 + currentHeading) - desiredHeading;
        }

        if (Math.abs(diff) <= HEADING_TOLERANCE_DEGREES)
            return 0;
        else
            // Return positive if the currentHeading is to the right of the initial heading, otherwise negative
            return diff;
    }

    private double getHeading() {
        return v_sensor_gyro.getAngularOrientation().heading;
    }

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
*/
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
        return String.format("%.1f", normalizeDegrees(degrees));
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

    // Normalize the angle into the range [-180,180) /
    double normalizeDegrees(double degrees)
    {
        if (degrees >= 360.0) degrees -= 360.0;
        if (degrees < 0.0) degrees += 360.0;
        return degrees;
    }
    double degreesFromRadians(double radians) {
        return radians * 180.0 / Math.PI;
    }

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
