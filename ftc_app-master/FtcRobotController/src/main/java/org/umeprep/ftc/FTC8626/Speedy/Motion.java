package org.umeprep.ftc.FTC8626.Speedy;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

// for telemetry.addData...
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.umeprep.ftc.FTC8626.Speedy.autonomous.DriveMoveDirection;
import org.umeprep.ftc.FTC8626.Speedy.autonomous.DriveTurnDirection;
/**
 * Created by Andre on 3/3/2016.
 */
public final class Motion {
    private final int ANDYMARK_MOTORS_TICKS_PER_REVOLUTION = 1120;
    private final int DISTANCE_ERROR_TOLERANCE_IN_TICKS = 50;
    private final int WHEEL_DIAMETER = 4;
    private final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

    private final double HEADING_TOLERANCE_DEGREES = 1.1;

    // Adjust the MOVE_POWER_FACTOR based on the smoothness of floor surface
    private final double DRIVE_SLOW_POWER = .15;
    private final double DEFAULT_DRIVE_MOTOR_POWER = .8;
    private final double MOVE_POWER_FACTOR = .75;   // Multiplier to reduce the actual power
    private final double SLOW_MOVE_POWER_FACTOR = .5;
    private final double MOVE_POWER_HEADING_ADJUSTMENT = .008;

    private final double TURN_POWER = .7; //.15
    private final double SLOW_TURN_POWER = .1;
    private final double SUPER_SLOW_TURN_POWER = .8;
    private final double TURN_POWER_FACTOR = 1.5;

    private Navigation navigation;
    //private double lastDesiredHeading;

    private DcMotor motorRight;
    private DcMotor motorLeft;

    public Motion(Navigation navigationParameter, DcMotor motorRightParameter,DcMotor motorLeftParameter){
        motorRight = motorRightParameter;
        motorLeft = motorLeftParameter;
        navigation = navigationParameter;
    }

    public Motion(DcMotor motorRightParameter,DcMotor motorLeftParameter){
        motorRight = motorRightParameter;
        motorLeft = motorLeftParameter;
    }

    public void simpleMoveBackwards() {
        //double movePower = Range.clip(DEFAULT_DRIVE_MOTOR_POWER * SLOW_MOVE_POWER_FACTOR, -1, 1);
        setMotorPower(.2);
    }

    public void move(DriveMoveDirection robotMoveDirection, double distanceInInches, double lastDesiredHeading) throws InterruptedException {
        double movePower = DEFAULT_DRIVE_MOTOR_POWER;

        double initialHeading = lastDesiredHeading;
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

        if (distanceInInches > 17) {
            // Adjust the power based on the factor (change based on smoothness of floor surface)

            initialMovePower = Range.clip(movePower * MOVE_POWER_FACTOR, -1, 1);
            //telemetry.addData("lastDesired","heading: " + lastDesiredHeading);

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

        ElapsedTime moveElapsedTime = new ElapsedTime();
        moveElapsedTime.reset();

        while (Math.abs(rightPositionDifference) > DISTANCE_ERROR_TOLERANCE_IN_TICKS ||
                Math.abs(leftPositionDifference) > DISTANCE_ERROR_TOLERANCE_IN_TICKS) {
        //||  (timeout > 0 && moveElapsedTime.time() > timeout))  {

            double headingDifference = navigation.getHeadingDifference(lastDesiredHeading);

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

        stopDriveMotors();
        Thread.sleep(300);
    }

    public double turn(DriveTurnDirection direction, double turnAngle, double lastDesiredHeading) throws InterruptedException {

        motorRight.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        motorLeft.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);

        turnAngle = Range.clip(turnAngle, 0, 180);
        turnAngle = (direction == DriveTurnDirection.Left) ? -turnAngle : turnAngle;

        // Adjust the turnPowerFactor based on the smoothness of floor surface
        double turnPower = Range.clip(TURN_POWER * TURN_POWER_FACTOR, -1, 1);

        // Determine how far off the robot is currently
        //telemetry.addData("lastDesired","heading: " + lastDesiredHeading);
        double headingDrift = navigation.getAngleCorrectionAfterDrift(direction, lastDesiredHeading);
        //telemetry.addData("drift", "in heading: " + headingDrift);

        // Correct turn amount for existing drift
        if (Math.abs(headingDrift) > HEADING_TOLERANCE_DEGREES) {
            if (direction == DriveTurnDirection.Right)
                turnAngle = Utility.normalizeDegrees(turnAngle + headingDrift);  // Drift will be a negative number in some cases
            else
                turnAngle = Utility.normalizeDegrees(turnAngle - headingDrift);  // Drift will be a negative number in some cases
        }

        double desiredHeading = Utility.normalizeDegrees(lastDesiredHeading + turnAngle); //currentHeading + turnAngle);
        //telemetry.addData("desired","heading: " + desiredHeading);

        // Reset the last desired heading with the new direction
        lastDesiredHeading = desiredHeading;

        double headingDifference = navigation.getHeadingDifference(desiredHeading);
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

            headingDifference = navigation.getHeadingDifference(desiredHeading);
            Thread.sleep(20); // simulate the slower looping of a TeleOp mode - 50 times per second
        }

        stopDriveMotors();
        Thread.sleep(300);  // Rest the motors after turning

        return lastDesiredHeading;
    }

    public void setMotorPower(double power) {
        setMotorPower(power, power);
    }

    public void setMotorPower(double rightPower, double leftPower) {
        motorRight.setPower(rightPower);
        motorLeft.setPower(leftPower);
    }

    public void stopDriveMotors() {
        setMotorPower(0);
    }


}

