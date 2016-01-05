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
import org.swerverobotics.library.SynchronousOpMode;
import org.swerverobotics.library.TelemetryDashboardAndLog;
import org.swerverobotics.library.interfaces.EulerAngles;
import org.swerverobotics.library.interfaces.IBNO055IMU;
import org.swerverobotics.library.interfaces.IFunc;
import org.swerverobotics.library.interfaces.II2cDeviceClientUser;
import org.swerverobotics.library.interfaces.Position;
import org.swerverobotics.library.interfaces.TeleOp;
import org.swerverobotics.library.interfaces.Velocity;
import org.umeprep.ftc.FTC8626.Speedy.autonomous.DriveMoveDirection;
import org.umeprep.ftc.FTC8626.Speedy.autonomous.DriveTurnDirection;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.TouchSensor;
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

    public boolean v_warning_generated = false;
    public String v_warning_message;
    // Here we have state we use for updating the dashboard. The first of these is important
    // to read only once per update, as its acquisition is expensive. The remainder, though,
    // could probably be read once per item, at only a small loss in display accuracy.
    EulerAngles angles;
    Position position;
    int loopCycles;
    int i2cCycles;
    double ms;
    private DcMotor motorRight;
    private DcMotor motorLeft;
    private TouchSensor v_sensor_touch;
    private OpticalDistanceSensor v_sensor_ods;
    private IBNO055IMU v_sensor_gyro;
    private ElapsedTime elapsed    = new ElapsedTime();
    private IBNO055IMU.Parameters   parameters = new IBNO055IMU.Parameters();

    //Double turnDuration = 1.085;

    /**
     * Constructor
     */
/*
    public AutonomousTeleOp() {
        telemetry = new TelemetryDashboardAndLog();
        telemetry.addData("Robot habla", "hola");
    }
*/
    @Override
    public void main() throws InterruptedException {

        //try {
            initializeRobot();

            // Set up our dashboard computations
            composeDashboard();

            //telemetry.addData("Begin", "say something");

//            DisplayTelemetryOnPhone();

            waitForStart();


            //makeSomeMoves();
            //makeSomeOtherMoves();
            //DisplayTelemetryOnPhone();

        while (opModeIsActive())
        {
            /*
            if (this.updateGamepads()) {

            }
            */
            telemetry.update();
            this.idle();
        }

        //}
/*        catch (Exception ex) {
            telemetry.addData("Error", "Error: " + ex.getMessage());
        }
        finally {
            DisplayTelemetryOnPhone();
        }
*/
    }

    private void DisplayTelemetryOnPhone() throws InterruptedException {
        while (opModeIsActive())
        {
            telemetry.update();
            idle();
        }
    }

    private void initializeRobot() {
        //telemetry.addData("Robot says","Let's ride.");

        InitializeMotors();

        InitializeSensors();

    }


    private void InitializeSensors() {

        HardwareMap.DeviceMapping<I2cDevice> i2cDeviceList = hardwareMap.i2cDevice;

        for (I2cDevice item : i2cDeviceList) {
            String listDeviceName = item.getDeviceName();
            telemetry.addData(listDeviceName, "I2C Device class: " + item.getClass());
        }

/*
        try {
            v_sensor_touch = hardwareMap.touchSensor.get("Touch_1");
        } catch (Exception p_exeception) {
            //m_warning_message("Touch_1");
            //DbgLog.msg (p_exeception.getLocalizedMessage ());

            v_sensor_touch = null;
        }

        try {
            v_sensor_ods = hardwareMap.opticalDistanceSensor.get("ODS");
        } catch (Exception p_exception) {
           // m_warning_message("ODS");
            //DbgLog.msg (p_exception.getLocalizedMessage ());

            v_sensor_touch = null;
        }
*/
        //InitializeSensorGyro();
    }

    private void InitializeSensorGyro() {
        //telemetry.addData("Robot says", "IMU: adding parameters");

        parameters.angleunit      = IBNO055IMU.ANGLEUNIT.DEGREES;
        parameters.accelunit      = IBNO055IMU.ACCELUNIT.METERS_PERSEC_PERSEC;
        parameters.temperatureUnit = IBNO055IMU.TEMPUNIT.FARENHEIT;
        parameters.loggingEnabled = false;
        parameters.loggingTag     = "Gyro";
        parameters.pitchmode      = IBNO055IMU.PITCHMODE.ANDROID;

        telemetry.addData("Robot says", "IMU: added parameters");

        I2cDevice device = hardwareMap.i2cDevice.get("I2cDevice");
        v_sensor_gyro = ClassFactory.createAdaFruitBNO055IMU(this, device, parameters);

        telemetry.addData("Robot says", "IMU Device created");

        v_sensor_gyro.startAccelerationIntegration(new Position(), new Velocity());
        telemetry.addData("Robot says", "IMU acceleration started");


        double temp = v_sensor_gyro.getTemperature();
        telemetry.addData("Robot says", "IMU temp: " + temp);

        boolean isGyroCalibrated = v_sensor_gyro.isGyroCalibrated();
        telemetry.addData("gyro","calibrated: " + isGyroCalibrated);
    }


    private void InitializeMotors() {
        motorLeft = hardwareMap.dcMotor.get("Wheel 1");   // Get the name of the real motors
        motorRight = hardwareMap.dcMotor.get("Wheel 2");  // Get the name of the real motors
        motorLeft.setDirection(DcMotor.Direction.REVERSE);
    }

    private void makeSomeMoves() throws InterruptedException {

        telemetry.addData("makeSomeMoves", "Starting move forward");

            double duration = 1.2;
            move(DriveMoveDirection.Forward, .3, duration);
        //      telemetry.addData("makeSomeMoves", "After move forward");

        //move(DriveMoveDirection.Backward, .3, duration);

        //telemetry.addData("makeSomeMoves3", "After move backward");

        Thread.sleep(1000);

        //testTurn(0);

        turn(DriveTurnDirection.Left, 180);

        move(DriveMoveDirection.Forward, .3, 1);
/*
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

    private void makeSomeOtherMoves() throws InterruptedException {

        telemetry.addData("makeSomeOtherMoves", "Moving Forward");

            double duration = 2;
            move(DriveMoveDirection.Forward, .3,duration);

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
        turnSpecificTime(direction, moveAngle); //, turnDuration);
    }

    private void turnSpecificTime(DriveTurnDirection direction, float moveAngle)
    //, double duration)
            throws InterruptedException {

        double angle = Range.clip(moveAngle, 0, 180);

        // The following codewill be replaced by the gyro if we get it working
        double rightPower = .5;
        double leftPower = .5;

        // Translate the move Angle into an amount of right or left turn between 1 and -1
        if (direction == DriveTurnDirection.Left)
            leftPower = -rightPower;
        else
            rightPower = -leftPower;

        double duration = moveAngle / 180;
        //duration = 0 - moveAngle / 180;

        // clip the right/left values so that the values never exceed +/- 1
        rightPower = Range.clip(rightPower, -1, 1);
        leftPower = Range.clip(leftPower, -1, 1);

        // write the values to the motors
        setMotorPower(rightPower, leftPower);
        continueAction(duration);
    }

/*
    private void testTurn(double seed) throws InterruptedException {
        //double interval = 1;

        for (int diff = 1; diff <= 5; diff++) {
            double turnAmount = seed + turnDuration; // interval * diff;
            turnSpecificTime(DriveTurnDirection.Right, 180, turnAmount);
            telemetry.addData("testTurn", "Turn amount: " + turnAmount);
            Thread.sleep(1000);
        }
    }
*/

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


    // Swerve code
//----------------------------------------------------------------------------------------------
// dashboard configuration
//----------------------------------------------------------------------------------------------

    void composeDashboard()
    {
        // The default dashboard update rate is a little too slow for our taste here, so we update faster
        telemetry.setUpdateIntervalMs(200);

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles     = v_sensor_gyro.getAngularOrientation();
            position   = v_sensor_gyro.getPosition();

            // The rest of this is pretty cheap to acquire, but we may as well do it
            // all while we're gathering the above.
            loopCycles = getLoopCount();
            i2cCycles  = ((II2cDeviceClientUser) v_sensor_gyro).getI2cDeviceClient().getI2cCycleCount();
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
                    public Object value()
                    {
                        return formatRate(ms / loopCycles);
                    }
                }),
                telemetry.item("i2c cycle rate: ", new IFunc<Object>()
                {
                    public Object value()
                    {
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
        while (degrees >= 180.0) degrees -= 360.0;
        while (degrees < -180.0) degrees += 360.0;
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
