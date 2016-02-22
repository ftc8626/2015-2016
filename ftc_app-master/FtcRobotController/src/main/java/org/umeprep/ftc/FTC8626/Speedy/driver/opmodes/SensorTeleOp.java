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
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.swerverobotics.library.ClassFactory;
import org.swerverobotics.library.interfaces.EulerAngles;
import org.swerverobotics.library.interfaces.IBNO055IMU;
import org.swerverobotics.library.interfaces.Position;
import org.swerverobotics.library.interfaces.TeleOp;
import org.swerverobotics.library.interfaces.Velocity;

import java.util.Iterator;


/**
 * TeleOp Mode
 * <p/>
 * Enables control of the robot via the gamepad
 */
@TeleOp(name="SensorTeleOp", group="FTC8626")
public class SensorTeleOp extends OpMode {

    DcMotor motorRight;
    DcMotor motorLeft;

   // private TouchSensor v_sensor_touch;
    private OpticalDistanceSensor v_sensor_ods;

    private IBNO055IMU              imu;
    private ElapsedTime elapsed    = new ElapsedTime();
    private IBNO055IMU.Parameters   parameters = new IBNO055IMU.Parameters();

    // Here we have state we use for updating the dashboard. The first of these is important
    // to read only once per update, as its acquisition is expensive. The remainder, though,
    // could probably be read once per item, at only a small loss in display accuracy.
    EulerAngles angles;
    Position position;
    int loopCycles;
    int i2cCycles;
    double ms;

    // OpMode management from Swerve code SychronousOpMode.java
    private volatile boolean                started;
    private volatile boolean                stopRequested;

    // position of the arm servo.
    Servo dumpClimbers;
    double dumpClimbersInitPosition;
    double dumpClimbersUpPosition;
    double dumpClimbersDownPosition;

    // amount to change the tape measure up down servo position by
    Servo tapeMeasureUpDown;
    double tapeMeasureUpDownPosition;
    double tapeMeasureUpDownDelta = 0.1;

    public boolean v_warning_generated = false;
    public String v_warning_message;

    /**
     * Constructor
     */
    public SensorTeleOp() {

    }

    /*
     * Code to run when the op mode is first enabled goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */
    @Override
    public void init() {

        telemetry.addData("Robot says", "IMU: adding parameters");

        parameters.angleUnit      = IBNO055IMU.ANGLEUNIT.DEGREES;
        parameters.accelUnit      = IBNO055IMU.ACCELUNIT.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        parameters.loggingTag     = "Gyro";

        telemetry.addData("Robot says", "IMU: added parameters");

        try {

/*
            HardwareMap.DeviceMapping<I2cDevice> i2cDeviceList = hardwareMap.i2cDevice;
            for (I2cDevice item : i2cDeviceList)
            {
                String listDeviceName = item.getDeviceName();
                telemetry.addData("Robot says", "I2C Device list: " + listDeviceName);
*/
                I2cDevice device = hardwareMap.i2cDevice.get("I2cDevice");
//                I2cDevice device = hardwareMap.i2cDevice.get("Gyro");
//                String actualDeviceName = device.getDeviceName();
//                telemetry.addData("Robot says", "I2C Device retrieved: " + actualDeviceName);
//            }

            //telemetry.addData("Robot says", "IMU: creating IMU");

            imu = ClassFactory.createAdaFruitBNO055IMU(this, device, parameters);
                    //device, parameters);
//            imu = ClassFactory.createAdaFruitBNO055IMU(hardwareMap.i2cDevice.get("I2cDevice"), parameters);

            //telemetry.addData("Robot says", "IMU temp: " + imu.getTemperature());

        }
        catch (Exception ex)
        {
            telemetry.addData("Robot says", "Error: " + ex.getMessage());
        }

        telemetry.addData("Robot says", "IMU Device created");

        imu.startAccelerationIntegration(new Position(), new Velocity());
        telemetry.addData("Robot says", "IMU acceleration started");

        double temp = imu.getTemperature();
        telemetry.addData("Robot says", "IMU temp: " + temp);

        // Set up our dashboard computations
        //composeDashboard();

        //
        // Connect the sensors.
        //

        //imu = ClassFactory.createAdaFruitBNO055IMU(hardwareMap.i2cDevice.get("Gyro"), parameters);

        // Enable reporting of position using the naive integrator
        //imu.startAccelerationIntegration(new Position(), new Velocity());

        //try {
            //v_sensor_gyro = hardwareMap.i2cDevice.get("Gyro");
        //} catch (Exception p_exeception) {
          //  m_warning_message("Gyro");
            //v_sensor_touch = null;
        //}

        //try {
            //v_sensor_touch = hardwareMap.touchSensor.get("Touch_1");
        //} catch (Exception p_exeception) {
            //m_warning_message("Touch_1");
           // v_sensor_touch = null;
        //}

       //try {
            //v_sensor_ods = hardwareMap.opticalDistanceSensor.get("ODS");
       //} catch (Exception p_exeception) {
          //  m_warning_message("ODS");
            //v_sensor_ods = null;
       //}

/*
        if (a_ods_light_detected() > 0) {
            telemetry.addData("Robot says", "ODS: " + a_ods_light_detected());
        } else {
            telemetry.addData("Robot says", "ODS: No light detected");
        }
*/
        //telemetry.addData("Robot says", "gyro: " + imu.getGravity());

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
		/*
		motorLeft = hardwareMap.dcMotor.get("motor_1");

		motorRight = hardwareMap.dcMotor.get("motor_2");
		motorLeft.setDirection(DcMotor.Direction.REVERSE);

		// write the values to the motors
		motorRight.setPower(0);
		motorLeft.setPower(0);


		// Servo code
		servoTapeMeasureUpDown = hardwareMap.servo.get("servo_1");
		//dumpClimbers = hardwareMap.servo.get("servo_6");

		// assign the starting position of the servos
		tapeMeasureUpDownPosition = 1;

		//dumpClimberArmPosition = 0.2;

		// write position values to the wrist and claw servo
		servoTapeMeasureUpDown.setPosition(tapeMeasureUpDownPosition);
		//dumpClimbers.setPosition(dumpClimberArmPosition);

*/

        //try
        //{
        //	delay(2);
        //}
        //catch (InterruptedException ex) {}
        //tapeMeasureUpDownPosition = .5;
        //servoTapeMeasureUpDown.setPosition(tapeMeasureUpDownPosition);
        //try
        //{
        //	delay(2);
        //}
        //catch (InterruptedException ex) {}

        //telemetry.addData("","Welcome Driver");
        //telemetry.addData("Robot says", "Hi");
    }

    /*
     * This method will be called repeatedly in a loop
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
     */
    @Override
    public void loop() {

        // Loop and update the dashboard
        /*
        while (opModeIsActive())
        {
            telemetry.update();
            idle();
        }
        */

        //telemetry.addData("Text", "Position:" + position);
/*
        if (is_touch_sensor_pressed()) {
            telemetry.addData("Robot says", "Touched: " + is_touch_sensor_pressed());
        }

        //if (is_touch_sensor_pressed()) {
            if (a_ods_light_detected() > 0) {
                telemetry.addData("Robot says", "ODS: " + a_ods_light_detected());
            } else {
                telemetry.addData("Robot says", "ODS: No light detected");
            }
        //}

        if (is_touch_sensor_pressed()) {
            if (adaFruitBNO055IMU.isGyroCalibrated()) {
                telemetry.addData("Robot says", "Gyro is calibrated");
            } else {
                telemetry.addData("Robot says", "Gyro is not calibrated");
            }
            telemetry.addData("Robot says", "Gyro position: " + adaFruitBNO055IMU.getPosition());
        }

*/
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
/*
		float throttle = -gamepad1.left_stick_y;
		float direction = gamepad1.left_stick_x;
		float right = throttle + direction;
		float left = throttle - direction;

		// clip the right/left values so that the values never exceed +/- 1
		right = Range.clip(right, -1, 1);
		left = Range.clip(left, -1, 1);

		// scale the joystick value to make it easier to control
		// the robot more precisely at slower speeds.
		right = (float)scaleInput(right);
		left =  (float)scaleInput(left);
		
		// write the values to the motors
		motorRight.setPower(right);
		motorLeft.setPower(left);


		// Set servos
		// update the position of the claw
		if (gamepad1.dpad_up) {
			//tapeMeasureUpDownPosition += tapeMeasureUpDownDelta;
			tapeMeasureUpDownPosition = .7;
		}

		if (gamepad1.dpad_down) {
			//tapeMeasureUpDownPosition -= tapeMeasureUpDownDelta;
			tapeMeasureUpDownPosition = 1;
		}

		//tapeMeasureUpDownPosition = .7;
		servoTapeMeasureUpDown.setPosition(tapeMeasureUpDownPosition);
		// clip the position values so that they never exceed their allowed range.
		//tapeMeasureUpDownPosition = Range.clip(armPosition, ARM_MIN_RANGE, ARM_MAX_RANGE);
		//clawPosition = Range.clip(clawPosition, CLAW_MIN_RANGE, CLAW_MAX_RANGE);

		// write position values to the wrist and claw servo
		//servoTapeMeasureUpDown.setPosition(tapeMeasureUpDownPosition);
		//dumpClimbers.setPosition(dumpClimberArmPosition);

		//try
		//{
			//makeSomeMoves();
		//}
		//catch (InterruptedException ex) {}
		/*
		 * Send telemetry data back to driver station. Note that if we are using
		 * a legacy NXT-compatible motor controller, then the getPower() method
		 * will return a null value. The legacy NXT-compatible motor controllers
		 * are currently write only.
		 */
//        telemetry.addData("Text", "*** Robot Data***");
//        telemetry.addData("left tgt pwr",  "left  pwr: " + String.format("%.2f", left));
//        telemetry.addData("right tgt pwr", "right pwr: " + String.format("%.2f", right));


    }

    private void delay(int seconds) throws InterruptedException {

        int moveDurationInSeconds = seconds;
        int moveDuration = moveDurationInSeconds * 1000;  //# of Seconds x 1000 -> Sleep needs milliseconds, 2 seconds = 2000 milliseconds
        Thread.sleep(moveDuration);   //Thread.Sleep may experience an error while running so it can "throw an exception"

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
    double scaleInput(double dVal) {
        double[] scaleArray = {0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00};

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

    boolean is_touch_sensor_pressed()

    {
        boolean l_return = false;

        //if (v_sensor_touch != null) {
          //  l_return = v_sensor_touch.isPressed();
       // }

        return l_return;

    } // is_touch_sensor_pressed

    //--------------------------------------------------------------------------
    //
    // a_ods_light_detected
    //

    /**
     * Access the amount of light detected by the Optical Distance Sensor.
     */
    double a_ods_light_detected()

    {
        double l_return = 0;

        if (v_sensor_ods != null) {
           l_return = v_sensor_ods.getLightDetected();
            //l_return = v_sensor_ods.getLightDetectedRaw();
        }

        return l_return;

    } // a_ods_light_detected

    //----------------------------------------------------------------------------------------------
    // dashboard configuration
    //----------------------------------------------------------------------------------------------

   /* void composeDashboard()
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
            angles     = imu.getAngularOrientation();
            position   = imu.getPosition();

            // The rest of this is pretty cheap to acquire, but we may as well do it
            // all while we're gathering the above.
            loopCycles = getLoopCount();
            i2cCycles  = ((II2cDeviceClientUser) imu).getI2cDeviceClient().getI2cCycleCount();
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
                        return decodeStatus(imu.getSystemStatus());
                    }
                }),
                telemetry.item("calib: ", new IFunc<Object>()
                {
                    public Object value()
                    {
                        return decodeCalibration(imu.read8(IBNO055IMU.REGISTER.CALIB_STAT));
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
                telemetry.item("x: ", new IFunc<Object>()
                {
                    public Object value()
                    {
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
*/
    //----------------------------------------------------------------------------------------------
    // Utility
    //----------------------------------------------------------------------------------------------

    /** Normalize the angle into the range [-180,180) */
    double normalizeDegrees(double degrees) {
        while (degrees >= 180.0) degrees -= 360.0;
        while (degrees < -180.0) degrees += 360.0;
        return degrees;
    }

    double degreesFromRadians(double radians) {
        return radians * 180.0 / Math.PI;
    }

    /**
     * Turn a system status into something that's reasonable to show in telemetry
     */
    String decodeStatus(int status) {
        switch (status) {
            case 0:
                return "idle";
            case 1:
                return "syserr";
            case 2:
                return "periph";
            case 3:
                return "sysinit";
            case 4:
                return "selftest";
            case 5:
                return "fusion";
            case 6:
                return "running";
        }
        return "unk";
    }

    /**
     * Turn a calibration code into something that is reasonable to show in telemetry
     */
    String decodeCalibration(int status) {
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

    /*
     * Answer as to whether this opMode is active and the robot should continue onwards. If the
     * opMode is not active, synchronous threads should terminate at their earliest convenience.
     *
     * @return whether the OpMode is currently active. If this returns false, you should
     *         break out of the loop in your {@link #main()} method and return to its caller.
     * @see #main()
     * @see #isStarted()
     * @see #isStopRequested()
     */
    public final boolean opModeIsActive()
    {
        return !this.isStopRequested() && this.isStarted();
    }

    /**
     * Has the opMode been started?
     *
     * @return whether this opMode has been started or not
     * @see #opModeIsActive()
     * @see #isStopRequested()
     */
    public final boolean isStarted()
    {
        return this.started;
    }

    /**
     * Has the the stopping of the opMode been requested?
     *
     * @return whether stopping opMode has been requested or not
     * @see #opModeIsActive()
     * @see #isStarted()
     */
    public final boolean isStopRequested()
    {
        return this.stopRequested || Thread.currentThread().isInterrupted();
    }


}

