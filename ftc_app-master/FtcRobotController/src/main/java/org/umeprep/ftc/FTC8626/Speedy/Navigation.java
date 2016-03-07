package org.umeprep.ftc.FTC8626.Speedy;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDevice;

import org.swerverobotics.library.ClassFactory;
import org.swerverobotics.library.interfaces.IBNO055IMU;
import org.swerverobotics.library.interfaces.Position;
import org.swerverobotics.library.interfaces.Velocity;
import org.umeprep.ftc.FTC8626.Speedy.autonomous.DriveTurnDirection;

/**
 * Created by Andre on 3/3/2016.
 */
public class Navigation {
    private final double HEADING_TOLERANCE_DEGREES = 1.1;

    private IBNO055IMU v_sensor_gyro;
    private IBNO055IMU.Parameters parameters = new IBNO055IMU.Parameters();
    private OpMode opMode;
    private HardwareMap hardwareMap;

    public Navigation(OpMode opModeParameter, HardwareMap hardwareMapParameter){
        opMode = opModeParameter;
        hardwareMap = hardwareMapParameter;
    }

    public double getHeading() {
        return v_sensor_gyro.getAngularOrientation().heading;
    }

    public double getHeadingDifference(double desiredHeading) {
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

    public void InitializeSensorGyro() throws IllegalStateException, Exception {

        parameters.angleUnit      = IBNO055IMU.ANGLEUNIT.DEGREES;
        parameters.accelUnit      = IBNO055IMU.ACCELUNIT.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        parameters.loggingTag     = "Gyro";

        I2cDevice device = hardwareMap.i2cDevice.get("I2cDevice");

        try {
            v_sensor_gyro = ClassFactory.createAdaFruitBNO055IMU(opMode, device, parameters);
            v_sensor_gyro.startAccelerationIntegration(new Position(), new Velocity());

            opMode.telemetry.addData("IMU started","waiting 7 seconds");

        } catch (Exception ex) {
            opMode.telemetry.addData("Error creating IMU Device", ex.getMessage());
            throw ex;
        }

        // Sleep 15 seconds to allow gyro to fully calibrate
        Thread.sleep(7000);

        boolean isGyroCalibrated = v_sensor_gyro.isGyroCalibrated();
        if (!isGyroCalibrated) {
            throw new Exception("Gyro failed to calibrate");
        }
        else {
            opMode.telemetry.addData("IMU", "gyro calibrated");
        }
    }

    public double getAngleCorrectionAfterDrift(DriveTurnDirection direction, double desiredHeading) {

        double headingDifference = getHeadingDifference(desiredHeading);

        if (direction == DriveTurnDirection.Left)
            return headingDifference;
        else
            return -headingDifference;
    }
}
