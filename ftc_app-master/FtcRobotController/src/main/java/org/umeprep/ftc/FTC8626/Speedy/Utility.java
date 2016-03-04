package org.umeprep.ftc.FTC8626.Speedy;

/**
 * Created by Andre on 1/9/2016.
 */
public class Utility {
    /*
 * This method scales the joystick input so for low joystick values, the
 * scaled value is less than linear.  This is to make it easier to drive
 * the robot more precisely at slower speeds.
 */
    public static double scaleInput(double dVal)  {
        double[] scaleArray = { 0.0, 0.04, 0.07, 0.10, 0.13, 0.16, 0.20, 0.24,
                0.28, 0.32, 0.36, 0.42, 0.48, 0.56, 0.66, .80, 1.00 };

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

    // Normalize the angle into the range [-180,180) /
    public static double normalizeDegrees(double degrees)
    {
        if (degrees >= 360.0) degrees -= 360.0;
        if (degrees < 0.0) degrees += 360.0;
        return degrees;
    }
    public static double degreesFromRadians(double radians) {
        return radians * 180.0 / Math.PI;
    }


}
