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

import android.graphics.Color;

import org.swerverobotics.library.ClassFactory;
import org.swerverobotics.library.SynchronousOpMode;
import org.swerverobotics.library.interfaces.EulerAngles;
import org.swerverobotics.library.interfaces.IBNO055IMU;
import org.swerverobotics.library.interfaces.Position;
import org.swerverobotics.library.interfaces.TeleOp;
import org.swerverobotics.library.interfaces.Velocity;
import org.umeprep.ftc.FTC8626.Speedy.DebrisPusherDirection;
import org.umeprep.ftc.FTC8626.Speedy.autonomous.Category;
import org.umeprep.ftc.FTC8626.Speedy.autonomous.DriveMoveDirection;
import org.umeprep.ftc.FTC8626.Speedy.autonomous.DriveTurnDirection;
import org.umeprep.ftc.FTC8626.Speedy.autonomous.MenuChoices;
import org.umeprep.ftc.FTC8626.Speedy.autonomous.OptionMenu;
import org.umeprep.ftc.FTC8626.Speedy.autonomous.SingleSelectCategory;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Linear Tele Op Mode for Autonomous movement
 * <p/>
 */
@TeleOp(name="AutonomousTeleOp", group="FTC8626")
public class AutonomousTeleOp extends SynchronousOpMode {

    private final int ANDYMARK_MOTORS_TICKS_PER_REVOLUTION = 1120;
    private final int DISTANCE_ERROR_TOLERANCE_IN_TICKS = 5;
    private final int WHEEL_DIAMETER = 4;
    private final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

    private final double MOVE_HEADING_TOLERANCE_DEGREES = 1;
    private final double TURN_HEADING_TOLERANCE_DEGREES = .5;

    private final double DEFAULT_DRIVE_MOTOR_POWER = .5;

    // Adjust the MOVE_POWER_FACTOR based on the smoothness of floor surface
    private final double MOVE_POWER_FACTOR = .8;
    private final double SLOW_MOVE_POWER_FACTOR = .3;
    private final double MOVE_POWER_HEADING_ADJUSTMENT = .002;

    private final double TURN_POWER_FACTOR = 1.5;

    private final String ALLIANCE = "ALLIANCE";
    private final String ALLIANCE_RED = "RED";
    private final String ALLIANCE_BLUE = "BLUE";
    private final String STARTING_POSITION = "STARTING_POSITION";
    private final String STARTING_POSITION_LEFT = "LEFT";
    private final String STARTING_POSITION_RIGHT = "RIGHT";
    //public boolean v_warning_generated = false;
    //public String v_warning_message;

    // Here we have state we use for updating the dashboard. The first of these is important
    // to read only once per update, as its acquisition is expensive. The remainder, though,
    // could probably be read once per item, at only a small loss in display accuracy.
    //EulerAngles angles;
    //Position position;
    //int loopCycles;
    //int i2cCycles;
    //double ms;

    // Motor variables
    private DcMotor motorRight;
    private DcMotor motorLeft;
    private DcMotor motorHook;

    // Servo variables
    private Servo servoTapeMeasureUpDown;
    private Servo servoClimberDumper;
    private Servo servoDebrisPusherRight;
    private Servo servoDebrisPusherLeft;
    private Servo servoDebrisPusherMiddle;
    //private Servo servoButtonPusher;

    // Menu variables
    private OptionMenu menu;

    //
    // Sensor variables
    //
    private OpticalDistanceSensor v_sensor_distance;
    private ColorSensor v_sensor_color;
    //  private TouchSensor v_sensor_touch;

       // Gyro variables
    private IBNO055IMU v_sensor_gyro;
    private ElapsedTime elapsed = new ElapsedTime();
    private IBNO055IMU.Parameters parameters = new IBNO055IMU.Parameters();

    public AutonomousTeleOp()  {

        //this.init();
/*
        try {
            servoTapeMeasureUpDown = hardwareMap.servo.get("Hook Control");
//            servoClimberDumper = hardwareMap.servo.get("Climber Dumper");
//            servoDebrisPusherRight = hardwareMap.servo.get("Debris Pusher Right");
//            servoDebrisPusherLeft = hardwareMap.servo.get("Debris Pusher Left");
//            servoDebrisPusherMiddle = hardwareMap.servo.get("Debris Pusher Middle");

            //servoButtonPusher = hardwareMap.servo.get("Button Pusher");

//            servoClimberDumper.setPosition(0);

//            setDebrisPusher(DebrisPusherDirection.Down, false);
            //servoButtonPusher.setPosition(.7);

            servoTapeMeasureUpDown.setPosition(.5);
            Thread.sleep(1000);
            servoTapeMeasureUpDown.setPosition(.3);
        }
        catch (Exception ex) {
            telemetry.addData("error",ex.getMessage());
            telemetry.update();
        }
        */
    }

    @Override
    public void main() throws InterruptedException {

        try {
            initializeRobot();
            //composeDashboard();

            telemetry.update();

            waitForStart();

            makeSomeMoves();

        } catch (Exception ex) {

            telemetry.addData("Error", ex.getMessage());

        } finally {

            telemetry.update();
            shutDownRobot();
        }
    }


    private void initializeRobot() throws InterruptedException {

        InitializeMenu();
        InitializeMotors();
        InitializeSensors();
        InitializeServos();
    }

    private void InitializeMenu() throws InterruptedException {

        // This menu code was shared with us by team 4290 lasarobotics (thanks!)
        OptionMenu.Builder builder = new OptionMenu.Builder(hardwareMap.appContext);

        //Setup a SingleSelectCategory
        SingleSelectCategory alliance = new SingleSelectCategory(ALLIANCE);
        alliance.addOption(ALLIANCE_RED);
        alliance.addOption(ALLIANCE_BLUE);
        builder.addCategory(alliance);

        //Setup a SingleSelectCategory
        SingleSelectCategory startPosition = new SingleSelectCategory(STARTING_POSITION);
        startPosition.addOption(STARTING_POSITION_LEFT);
        startPosition.addOption(STARTING_POSITION_RIGHT);
        builder.addCategory(startPosition);
/*
        //Setup a TextCategory
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

//        DisplayTelemetryOnPhone();
    }

    private void InitializeSensors() {
/*
        HardwareMap.DeviceMapping<I2cDevice> i2cDeviceList = hardwareMap.i2cDevice;

        int n = 0;
        for (I2cDevice item : i2cDeviceList) {
            n = n + 1;
            String listDeviceName = item.getDeviceName();
            telemetry.addData(listDeviceName, "I2C Device class: " + item.getDeviceName());
            telemetry.addData("counter", n);
        }

        telemetry.update();
 */
/*  Keep this in case we need the touch sensor to prevent collision damage on the button pusher
        try {
            v_sensor_touch = hardwareMap.touchSensor.get("Touch_1");
        } catch (Exception p_exeception) {
            //m_warning_message("Touch_1");
            //DbgLog.msg (p_exeception.getLocalizedMessage ());

            v_sensor_touch = null;
            sensorRGB = hardwareMap.colorSensor.get("mr");
        }

        try {
            v_sensor_color = hardwareMap.colorSensor.get("MR Color");
        } catch (Exception p_exception) {
            // m_warning_message("ODS");
            //DbgLog.msg (p_exception.getLocalizedMessage ());

            v_sensor_color = null;
        }
*/
        try {
            v_sensor_distance = hardwareMap.opticalDistanceSensor.get("ODS");
        } catch (Exception p_exception) {
           // m_warning_message("ODS");
            //DbgLog.msg (p_exception.getLocalizedMessage ());

            v_sensor_distance = null;
        }

        InitializeSensorGyro();
    }

    private void InitializeSensorGyro() {
        telemetry.addData("Robot says", "IMU: adding parameters");

        parameters.angleunit      = IBNO055IMU.ANGLEUNIT.DEGREES;
        parameters.accelunit      = IBNO055IMU.ACCELUNIT.METERS_PERSEC_PERSEC;
        parameters.temperatureUnit = IBNO055IMU.TEMPUNIT.FARENHEIT;
        parameters.loggingEnabled = false;
        parameters.loggingTag     = "Gyro";
        parameters.pitchmode      = IBNO055IMU.PITCHMODE.ANDROID;

        telemetry.addData("Robot says", "IMU: added parameters");

        I2cDevice device = hardwareMap.i2cDevice.get("I2cDevice");
        telemetry.addData("device name: ", device.getDeviceName());

        try {
            v_sensor_gyro = ClassFactory.createAdaFruitBNO055IMU(this, device, parameters);

            v_sensor_gyro.startAccelerationIntegration(new Position(), new Velocity());
            telemetry.addData("Robot says", "IMU acceleration started");

            double temp = v_sensor_gyro.getTemperature();
            telemetry.addData("Robot says", "IMU temp: " + temp);

            double initHeading = v_sensor_gyro.getAngularOrientation().heading;
            telemetry.addData("Robot says", "initHeading: " + initHeading);

            boolean isGyroCalibrated = v_sensor_gyro.isGyroCalibrated();
            telemetry.addData("gyro", "calibrated: " + isGyroCalibrated);

        } catch (Exception ex) {
            telemetry.addData("Robot says", "Error creating IMU Device");
        }
    }

    /**
     * Access the amount of light detected by the Optical Distance Sensor.
     */
    double getLightDetected() {
        double l_return = 0;

        if (v_sensor_distance != null) {
            l_return = v_sensor_distance.getLightDetected();
            //l_return = v_sensor_distance.getLightDetectedRaw();
        }
        return l_return;
    }

    private void InitializeMotors() {
        // Configuration for hook in front
        //motorLeft = hardwareMap.dcMotor.get("Wheel 1");   // Get the name of the real motors
        //motorRight = hardwareMap.dcMotor.get("Wheel 2");  // Get the name of the real motors
        //motorRight.setDirection(DcMotor.Direction.REVERSE);

        //Configuration for button pusher in front
        motorRight = hardwareMap.dcMotor.get("Wheel 1");   // Get the name of the real motors
        motorLeft = hardwareMap.dcMotor.get("Wheel 2");  // Get the name of the real motors
        motorRight.setDirection(DcMotor.Direction.REVERSE);

        // Hook
        motorHook = hardwareMap.dcMotor.get("Hook");
        motorHook.setDirection(DcMotor.Direction.REVERSE);

    }

    private void InitializeServos() throws InterruptedException {

        servoTapeMeasureUpDown = hardwareMap.servo.get("Hook Control");
        servoClimberDumper = hardwareMap.servo.get("Climber Dumper");
        servoDebrisPusherRight = hardwareMap.servo.get("Debris Pusher Right");
        servoDebrisPusherLeft = hardwareMap.servo.get("Debris Pusher Left");
        servoDebrisPusherMiddle = hardwareMap.servo.get("Debris Pusher Middle");

        //servoButtonPusher = hardwareMap.servo.get("Button Pusher");

        servoClimberDumper.setPosition(.05);

        servoTapeMeasureUpDown.setPosition(.8);
    }

    private void makeSomeMoves() throws Exception, InterruptedException {

        MenuChoices menuChoices = getMenuChoices();

        moveTowardBeacon(menuChoices);
/*
        prepareToPushButton();
        readBeaconColor();
*/
        dumpClimbers();

        pushBeaconButton(menuChoices);

        moveTowardFloorGoal(menuChoices);

        stopDriveMotors();
    }

    private void moveTowardFloorGoal(MenuChoices menuChoices) throws InterruptedException {

        if (menuChoices.getAlliance() == ALLIANCE_RED) {
            turn(DriveTurnDirection.Left, 78);
            move(DriveMoveDirection.Forward, .3, 10);
        }
        else {
            turn(DriveTurnDirection.Right, 78);
            move(DriveMoveDirection.Forward, .3, 10);
        }
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
            // Move the pusher up or down
            servoDebrisPusherRight.setPosition(.15);
            servoDebrisPusherLeft.setPosition(.9);

            // Move the pusher up before moving the middle brace out of the way
            Thread.sleep(500);
            servoDebrisPusherMiddle.setPosition(pusherMiddlePosition);

        } else {

            pusherMiddlePosition = .4;

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

        double dumperPostion = .1;
        servoClimberDumper.setPosition(dumperPostion);
        while (dumperPostion < .7) {
            dumperPostion += .1;
            servoClimberDumper.setPosition(dumperPostion);
            Thread.sleep(300);
        }

        Thread.sleep(1000);

        while (dumperPostion > .1) {
            dumperPostion -= .1;
            servoClimberDumper.setPosition(dumperPostion);
        }

    }

    private void pushBeaconButton(MenuChoices menuChoices) throws InterruptedException {
        if (menuChoices.getAlliance() == ALLIANCE_RED) {
            if (menuChoices.getStartingPosition() == STARTING_POSITION_RIGHT){
                move(DriveMoveDirection.Forward, .5, 1.5);
                move(DriveMoveDirection.Backward, .5, 10);
            }
            else {
                move(DriveMoveDirection.Forward, .5, 4);
                move(DriveMoveDirection.Backward, .5, 10);
            }

        }
        else {
            if (menuChoices.getStartingPosition() == STARTING_POSITION_LEFT){
                move(DriveMoveDirection.Forward, .5, 1.5);
                move(DriveMoveDirection.Backward, .5, 10);
            }
            else {
                move(DriveMoveDirection.Forward, .5, 4);
                move(DriveMoveDirection.Backward, .5, 10);
            }
        }

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
            else if (categoryName == STARTING_POSITION) {
                menuChoices.setStartingPosition(menu.selectedOption(categoryName));
            }
        }

        telemetry.addData(ALLIANCE, menuChoices.getAlliance());
        telemetry.addData(STARTING_POSITION, menuChoices.getStartingPosition());
        telemetry.update();

        return menuChoices;
    }

    private void moveTowardBeacon(MenuChoices menuChoices) throws InterruptedException {

        initializeDebrisPusher();

        if (menuChoices.getAlliance() == ALLIANCE_RED) {

            //Thread.sleep(1000);

            if (menuChoices.getStartingPosition() == STARTING_POSITION_RIGHT){
                move(DriveMoveDirection.Forward, DEFAULT_DRIVE_MOTOR_POWER, 18);
                turn(DriveTurnDirection.Left, 45);
                move(DriveMoveDirection.Forward, .5, 80);
                turn(DriveTurnDirection.Left, 41.5);
                move(DriveMoveDirection.Forward, .5, 5.7);

            }
            else {
                move(DriveMoveDirection.Forward, DEFAULT_DRIVE_MOTOR_POWER, 36);
                turn(DriveTurnDirection.Left, 43);
                move(DriveMoveDirection.Forward, .5, 52);
                turn(DriveTurnDirection.Left, 41.5);
                move(DriveMoveDirection.Forward, .5, 2.6);
            }
            //Thread.sleep(2000);

            //prepareToPushButton();
            //turn(DriveTurnDirection.Right, 90);
        }
        else { //After "else" Blue Alliance is activated -Blade

            if (menuChoices.getStartingPosition() == STARTING_POSITION_LEFT){
                move(DriveMoveDirection.Forward, DEFAULT_DRIVE_MOTOR_POWER, 18);
                turn(DriveTurnDirection.Right, 45);
                move(DriveMoveDirection.Forward, .5, 78);
                turn(DriveTurnDirection.Right, 40.5);
                move(DriveMoveDirection.Forward, .5, 5.7);
            }
            else {
                move(DriveMoveDirection.Forward, DEFAULT_DRIVE_MOTOR_POWER, 32);
                turn(DriveTurnDirection.Right, 45);
                move(DriveMoveDirection.Forward, .5, 48);
                turn(DriveTurnDirection.Right, 41.5);
                move(DriveMoveDirection.Forward, .5, 7.4);

            }
            //Thread.sleep(1000);

            //move(DriveMoveDirection.Forward, .5, 48);
            //Thread.sleep(2000);

            //prepareToPushButton();
            //turn(DriveTurnDirection.Left, 90);
        }

        //move(DriveMoveDirection.Forward, .5, 36); //96
    }

    private void readBeaconColor() {
        float hsvValues[] = {0,0,0};
        Color.RGBToHSV(v_sensor_color.red() * 8, v_sensor_color.green() * 8, v_sensor_color.blue() * 8, hsvValues);


        telemetry.addData("Clear", v_sensor_color.alpha());
        telemetry.addData("Red  ", v_sensor_color.red());
        telemetry.addData("Green", v_sensor_color.green());
        telemetry.addData("Blue ", v_sensor_color.blue());
        telemetry.addData("Hue", hsvValues[0]);
    }

    private void prepareToPushButton() throws InterruptedException {

        telemetry.addData("prepareToPushBotton", "starting");
        double lightDetected = getLightDetected();

        //telemetry.addData("getLightDetected: ", +lightDetected);
        //telemetry.update();

        // Move until about 3 inches away
        while (lightDetected < 80) {
            move(DriveMoveDirection.Forward, .3, 1);
            telemetry.addData("getLightDetected: ", + lightDetected);
            telemetry.update();
        }

        telemetry.addData("Yay! Light detected","");
        telemetry.update();
    }

    private void move(DriveMoveDirection robotMoveDirection, double distanceInInches) throws InterruptedException {
        double defaultMotorPower = DEFAULT_DRIVE_MOTOR_POWER;
        move(robotMoveDirection, defaultMotorPower, distanceInInches);
    }

    private void move(DriveMoveDirection robotMoveDirection, double movePower, double distanceInInches) throws InterruptedException {

        motorRight.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorLeft.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);

        //motorRight.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        //motorLeft.setMode(DcMotorController.RunMode.RESET_ENCODERS);

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
/*
        telemetry.addData("move", "Right position: " + currentRightPosition);
        telemetry.addData("move", "Left position: " + currentLeftPosition);
        telemetry.addData("move", "Target Right position: " + targetRightPosition);
        telemetry.addData("move", "Target Left position: " + targetLeftPosition);
        telemetry.addData("move", "DistanceInRevs: " + distanceInRevolutions);
        telemetry.addData("move", "DistanceInTicks: " + distanceInTicks);
        telemetry.update();
*/
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

        double initialHeading = getHeading();
        double currentHeading;

        double rightPositionDifference = motorRight.getCurrentPosition() - targetRightPosition;
        double leftPositionDifference = motorLeft.getCurrentPosition() - targetLeftPosition;

        while (Math.abs(rightPositionDifference) > DISTANCE_ERROR_TOLERANCE_IN_TICKS ||
                Math.abs(leftPositionDifference) > DISTANCE_ERROR_TOLERANCE_IN_TICKS)  {

            currentHeading = getHeading();
            double headingDifference = getMoveHeadingDifference(currentHeading, initialHeading);

/*            telemetry.addData("currentHeading: ", currentHeading);
            telemetry.addData("headingDiff: ", headingDifference);
*/
            telemetry.addData("move", "Right position: " + currentRightPosition);
            telemetry.addData("move", "Left position: " + currentLeftPosition);
            telemetry.addData("move", "Target Right position: " + targetRightPosition);
            telemetry.addData("move", "Target Left position: " + targetLeftPosition);
            telemetry.update();

            Thread.sleep(500);

            double rightPower = Range.clip(initialMovePower*MOVE_POWER_FACTOR, -1, 1);
            double leftPower = rightPower;
/*
            if (headingDifference == 1) {
                leftPower -= MOVE_POWER_HEADING_ADJUSTMENT;
                rightPower += MOVE_POWER_HEADING_ADJUSTMENT;
            } else if (headingDifference == -1) {
                leftPower += MOVE_POWER_HEADING_ADJUSTMENT;
                rightPower -= MOVE_POWER_HEADING_ADJUSTMENT;
            }
*/

            if (rightPositionDifference > DISTANCE_ERROR_TOLERANCE_IN_TICKS)
                rightPower = -rightPower;
            if (leftPositionDifference > DISTANCE_ERROR_TOLERANCE_IN_TICKS)
                leftPower = -leftPower;

            setMotorPower(rightPower, leftPower);

            //Thread.sleep(1000);

            rightPositionDifference = motorRight.getCurrentPosition() - targetRightPosition;
            leftPositionDifference = motorLeft.getCurrentPosition() - targetLeftPosition;
        }

        stopDriveMotors();
        Thread.sleep(300);
    }

    private void setMotorPower(double power) {
        setMotorPower(power, power);
    }

    private void setMotorPower(double rightPower, double leftPower) {
        //telemetry.addData("setMotorPower1", "Starting");
        //telemetry.addData("setMotorPower2", "LeftPower: " + leftPower); //motorLeft.getPower());
        //telemetry.addData("setMotorPower3", "Right power: " + rightPower); //motorRight.getPower());

        motorLeft.setPower(leftPower);
        motorRight.setPower(rightPower);
    }

    // Incorporate this if needed
    //update_telemetry (); // Update common telemetry
    //update_gamepad_telemetry ();

    private void stopDriveMotors() {
        setMotorPower(0);
    }

    private void stopServos() throws InterruptedException {
        servoClimberDumper.setPosition(0);
        servoTapeMeasureUpDown.setPosition(0);
        setDebrisPusher(DebrisPusherDirection.Down, false);
    }

    private void shutDownRobot() throws InterruptedException {
        stopDriveMotors();
        stopServos();
    }
/*
    private void floatRobot() {
        motorRight.setPowerFloat();
        motorLeft.setPowerFloat();
    }
*/

    private void turn(DriveTurnDirection direction, double turnAngle) throws InterruptedException {

        motorRight.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        motorLeft.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);

        turnAngle = Range.clip(turnAngle, 0, 180);
        turnAngle = (direction == DriveTurnDirection.Left) ? -turnAngle : turnAngle;

        // Adjust the turnPowerFactor based on the smoothness of floor surface
        double turnPower = .15 * TURN_POWER_FACTOR;

        double initialHeading = getHeading();
        double currentHeading = initialHeading;
        double newHeading = initialHeading + turnAngle;
        double desiredHeading =  normalizeDegrees(newHeading);

        double headingDifference = getTurnHeadingDifference(currentHeading, desiredHeading);

        while (headingDifference > TURN_HEADING_TOLERANCE_DEGREES) {

            if (Math.abs(headingDifference) < 60) {
                turnPower = .1 * TURN_POWER_FACTOR;
            } else if (Math.abs(headingDifference) < 30) {
                turnPower = .08 * TURN_POWER_FACTOR;
            }

            double desiredMinusCurrent = desiredHeading - currentHeading;
            if (desiredMinusCurrent > 180 || (desiredMinusCurrent < 0 && desiredMinusCurrent > -180)) {
            //if (direction == DriveTurnDirection.Left) {
//                telemetry.addData("Turn", "Turning left");
                motorRight.setPower(turnPower);
                motorLeft.setPower(0);
            } else if ((desiredMinusCurrent   > 0 && desiredMinusCurrent <= 180) || desiredMinusCurrent <= -180) {
//                telemetry.addData("Turn", "Turning right");
                // if the signal is to the right move right
                motorRight.setPower(0);
                motorLeft.setPower(turnPower);
            } else {
                stopDriveMotors();
                Thread.sleep(300);
            }

            currentHeading = getHeading();
            headingDifference = getTurnHeadingDifference(currentHeading, desiredHeading);

            telemetry.addData("Turn", "Initial heading: " + initialHeading);
            telemetry.addData("Turn", "Current heading: " + currentHeading);
            telemetry.addData("Turn", "Desired heading: " + desiredHeading);
            telemetry.addData("Turn", "Heading difference: " + headingDifference);
            telemetry.update();
        }

        stopDriveMotors();
        Thread.sleep(300);
    }

/*
    private void turn(DriveTurnDirection direction, float turnAngle) throws InterruptedException {

        motorRight.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        motorLeft.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);

        final double headingToleranceDegrees = 1;
        final long delayToAllowTurn = 20;

        double rightPower = .5;
        double leftPower = .5;
        double scaledPowerFactor = 1;

        double initialHeading = getHeading();
        telemetry.addData("Turn", "Initial heading: " + initialHeading);

        double currentHeading = initialHeading;
        double desiredHeading = 0;

        turnAngle = Range.clip(turnAngle, 0, 180);

        if (direction == DriveTurnDirection.Left)
            desiredHeading = normalizeDegrees(initialHeading - turnAngle);
        else
            desiredHeading = normalizeDegrees(initialHeading + turnAngle);

        double headingDifference = getTurnHeadingDifference(currentHeading, desiredHeading);

        while (headingDifference > headingToleranceDegrees) {

            // if the heading is greater than desired turn left
            rightPower = rightPower * scaledPowerFactor;
            leftPower = leftPower * scaledPowerFactor;

            telemetry.addData("Turn", "Desired heading: " + desiredHeading);
            telemetry.addData("Turn", "Current heading: " + currentHeading);
            telemetry.update();

            double desiredMinusCurrent = desiredHeading - currentHeading;
            if (desiredMinusCurrent > 180 || (desiredMinusCurrent < 0 && desiredMinusCurrent > -180)) {
                telemetry.addData("Turn", "Turning left");
                motorRight.setPower(rightPower);
                motorLeft.setPower(0);
            } else if ((desiredMinusCurrent   > 0 && desiredMinusCurrent <= 180) || desiredMinusCurrent <= -180) {
                telemetry.addData("Turn", "Turning right");
                // if the signal is to the right move right
                motorRight.setPower(0);
                motorLeft.setPower(leftPower);
            } else {
                telemetry.addData("Turn", "Off power");
                motorRight.setPower(0);
                motorLeft.setPower(0);
            }

            Thread.sleep(delayToAllowTurn);

            currentHeading = getHeading();
            headingDifference = getTurnHeadingDifference(currentHeading, desiredHeading);

            telemetry.addData("Turn", "Current heading: " + currentHeading);
            telemetry.addData("Turn", "Heading difference: " + headingDifference);
            telemetry.update();
        }

        // stop the motors
        motorRight.setPower(0);
        motorLeft.setPower(0);
    }
*/
    private int getMoveHeadingDifference(double currentHeading, double initialHeading) {
        double diff = currentHeading - initialHeading;
        double absDiff = Math.abs(diff);

        if (absDiff > 180) {
            if (initialHeading < currentHeading)
                diff = currentHeading - (360 + initialHeading);
            else
                diff = (360 + currentHeading) - initialHeading;
        }

        if (Math.abs(diff) <= MOVE_HEADING_TOLERANCE_DEGREES)
            return 0;
        else
            // Return 1 if the currentHeading is to the right of the initial heading, otherwise -1
            return (diff > 0) ? 1 : -1;
    }

    private double getTurnHeadingDifference(double currentHeading, double desiredHeading) {

        double absDiff = Math.abs(currentHeading - desiredHeading);

        if (absDiff > 180) {
            return (360 + currentHeading) - desiredHeading;
        } else {
            return absDiff;
        }
    }

    private double getHeading() {

        EulerAngles angles = v_sensor_gyro.getAngularOrientation();
        double currentHeading = angles.heading;

        return Range.clip(currentHeading, 0, 360);
    }

    /**
     * Mutate the warning message by ADDING the specified message to the current
     * message; set the warning indicator to true.
     * <p/>
     * A comma will be added before the specified message if the message isn't
     * empty.
     */
/*    void m_warning_message(String p_exception_message)

    {
        if (v_warning_generated) {
            v_warning_message += ", ";
        }
        v_warning_generated = true;
        v_warning_message += p_exception_message;

    } // m_warning_message
*/

    // Swerve code
//----------------------------------------------------------------------------------------------
// dashboard configuration
//----------------------------------------------------------------------------------------------

    void composeDashboard()
    {
        // The default dashboard update rate is a little too slow for our taste here, so we update faster
        telemetry.setUpdateIntervalMs(200);
/*
        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() {
            @Override
            public void run() {
                // Acquiring the angles is relatively expensive; we don't want
                // to do that in each of the three items that need that info, as that's
                // three times the necessary expense.
                angles = v_sensor_gyro.getAngularOrientation();
                position = v_sensor_gyro.getPosition();

                // The rest of this is pretty cheap to acquire, but we may as well do it
                // all while we're gathering the above.
                loopCycles = getLoopCount();
                i2cCycles = ((II2cDeviceClientUser) v_sensor_gyro).getI2cDeviceClient().getI2cCycleCount();
                ms         = elapsed.time() * 1000.0;
        }
        });
        
        telemetry.addLine(
                telemetry.item("loop count: ", new IFunc<Object>()
                {
                    public Object value()
                    {
                        return loopCycles;
                    }
                }),
                telemetry.item("i2c cycle count: ", new IFunc<Object>()
                {
                    public Object value()
                    {
                        return i2cCycles;
                    }
                }));

        telemetry.addLine(
                telemetry.item("loop rate: ", new IFunc<Object>()
                {
                    public Object value() {
                        return formatRate(ms / loopCycles);
                    }
                }),
                telemetry.item("i2c cycle rate: ", new IFunc<Object>()
                {
                    public Object value() {
                        return formatRate(ms / i2cCycles);
                    }
                }));

        telemetry.addLine(
                telemetry.item("status: ", new IFunc<Object>()
                {
                    public Object value()
                    {
                        return decodeStatus(v_sensor_gyro.getSystemStatus());
                    }
                }),
                telemetry.item("calib: ", new IFunc<Object>()
                {
                    public Object value()
                    {
                        return decodeCalibration(v_sensor_gyro.read8(IBNO055IMU.REGISTER.CALIB_STAT));
                    }
                }));

        telemetry.addLine(
                telemetry.item("heading: ", new IFunc<Object>()
                {
                    public Object value()
                    {
                        return formatAngle(angles.heading);
                    }
                }),
                telemetry.item("roll: ", new IFunc<Object>()
                {
                    public Object value()
                    {
                        return formatAngle(angles.roll);
                    }
                }),
                telemetry.item("pitch: ", new IFunc<Object>()
                {
                    public Object value()
                    {
                        return formatAngle(angles.pitch);
                    }
                }));

        telemetry.addLine(
                telemetry.item("x: ", new IFunc<Object>() {
                    public Object value() {
                        return formatPosition(position.x);
                    }
                }),
                telemetry.item("y: ", new IFunc<Object>()
                {
                    public Object value()
                    {
                        return formatPosition(position.y);
                    }
                }),
                telemetry.item("z: ", new IFunc<Object>()
                {
                    public Object value()
                    {
                        return formatPosition(position.z);
                    }
                }));
*/
    }

    String formatAngle(double angle)
    {
        return parameters.angleunit==IBNO055IMU.ANGLEUNIT.DEGREES ? formatDegrees(angle) : formatRadians(angle);
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
        String unit = parameters.accelunit== IBNO055IMU.ACCELUNIT.METERS_PERSEC_PERSEC
                ? "m" : "??";
        return String.format("%.2f%s", coordinate, unit);
    }

    //----------------------------------------------------------------------------------------------
    // Utility
    //----------------------------------------------------------------------------------------------

    /** Normalize the angle into the range [-180,180) */
    double normalizeDegrees(double degrees)
    {
        if (degrees >= 360.0) degrees -= 360.0;
        if (degrees < 0.0) degrees += 360.0;
        return degrees;
    }
    double degreesFromRadians(double radians)
    {
        return radians * 180.0 / Math.PI;
    }

    /** Turn a system status into something that's reasonable to show in telemetry */
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

    /** Turn a calibration code into something that is reasonable to show in telemetry */
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
}
