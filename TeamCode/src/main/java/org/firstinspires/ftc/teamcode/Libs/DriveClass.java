// change comments
/*
12/4/21 corrected motor / variable associations in calcDistance
*/
package org.firstinspires.ftc.teamcode.Libs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.HardwareProfile.HardwareProfile;


public class DriveClass {
    private final HardwareProfile robot;
    private final LinearOpMode opMode;
    private double RFPower;
    private double RRPower;
    private double LFPower;
    private double LRPower;
    public double startZAngle=0;

    //constructor
    public DriveClass(HardwareProfile myRobot, LinearOpMode myOpMode) {
        robot = myRobot;
        opMode = myOpMode;
        //getStartAngle();
    }

    /**
     * Method: driveStraight
     * @param power   - Power to drive
     * @param distance   - Distance to travel
     */
    public void driveStraight(double power, double distance) {
        boolean active = true;

        double LFStart = 0;
        double LRStart = 0;
        double RFStart = 0;
        double RRStart = 0;
        double coastMinimum = 5;
        double coastDistance = 0.8;
        double coastPower = 0.75;

        LFStart = robot.motorLF.getCurrentPosition();
        LRStart = robot.motorLR.getCurrentPosition();
        RFStart = robot.motorRF.getCurrentPosition();
        RRStart = robot.motorRR.getCurrentPosition();

        while(opMode.opModeIsActive() && active) {

            /*
             * Limit that value of the drive motors so that the power does not exceed 100%
             */
            RFPower = Range.clip(power, -1,1);
            LFPower = Range.clip(power, -1,1);
            RRPower = Range.clip(power, -1,1);
            LRPower = Range.clip(power, -1,1);

            /*
             * Apply power to the drive wheels
             */

            if(distance*coastDistance < coastMinimum) {
                if(Math.abs(distance-coastMinimum)<= Math.abs(calcDistance(RFStart, RRStart, LFStart, LRStart))){
                    RFPower *=0.65;
                    RRPower *=0.65;
                    LFPower *=0.65;
                    LRPower *=0.65;
                }
            }else {
                if (Math.abs(distance * coastDistance) <= Math.abs(calcDistance(RFStart, RRStart, LFStart, LRStart))) {
                    RFPower *= coastPower;
                    RRPower *= coastPower;
                    LFPower *= coastPower;
                    LRPower *= coastPower;
                }

            }
            setDrivePower(RFPower, RRPower, LFPower, RRPower);
            opMode.telemetry.addData("LF Start = ", LFStart);
            opMode.telemetry.addData("Distance = ", distance);
            opMode.telemetry.addData("Calculated Distance = ", calcDistance(RFStart, RRStart, LFStart, LRStart));
            opMode.telemetry.update();


            if(calcDistance(RFStart, RRStart, LFStart, LRStart) >= distance) active = false;
            opMode.idle();

        }   // end of while loop

        motorsHalt();

    } // close driveStraight method

    /**
     * Method: driveStraightPID
     * @param distance   - Distance to travel
     */
    public void driveStraightPID(double distance) {

        boolean active  = true;
        double error =distance;
        double errorFactor = 1;
        double deltaError = 0;
        double lastError = 0;
        double pidPower;
        double Cp = 0.03;
        double Ci = 0.05;
        double Cd = 0.1;
        double integral = 0.0;
        double maxSpeed = 0.65;
        int direction=1;

        if (distance<0) {
            direction = -1;
            distance = -distance;
        }

        double l1Start = robot.motorLF.getCurrentPosition();
        double l2Start = robot.motorLR.getCurrentPosition();
        double RFstart = robot.motorRF.getCurrentPosition();
        double r2Start = robot.motorRR.getCurrentPosition();

        error = distance - calcDistance(RFstart, r2Start, l1Start, l2Start);
        while (active && opMode.opModeIsActive()) {
            deltaError = lastError - error;

            // Clip motor speed
            pidPower = Range.clip((((Cp * error) + (Ci * integral) + (Cd * deltaError)) * maxSpeed * direction), -maxSpeed, maxSpeed);

            setDrivePower(pidPower, pidPower, pidPower, pidPower);

            lastError = error;

            error = distance - calcDistance(RFstart, r2Start, l1Start, l2Start);
            opMode.idle();

            if(error <= errorFactor) {
                active = false;
            }   // end of if(error <= errorFactor)
        }   // end of while Math.abs(error)
        motorsHalt();   // shutdown the drive motors

    } // close driveStraightPID method

    /**
     * Method: calcDistance
     * @param RFstart   - Right Front starting encoder value
     * @param RRStart   - Right Rear starting encoder value
     * @param LFStart   - Left Front starting encoder value
     * @param LRStart   - Left Rear starting encoder value
     */
    public double calcDistance(double RFstart, double RRStart, double LFStart, double LRStart){

        double distanceTraveled = 0;
        double r1Encoder = robot.motorRF.getCurrentPosition();
        double r2Encoder = robot.motorRR.getCurrentPosition();
        double l1Encoder = robot.motorLF.getCurrentPosition();
        double l2Encoder = robot.motorLR.getCurrentPosition();

        distanceTraveled = ((Math.abs(RFstart - r1Encoder) + Math.abs(RRStart - r2Encoder)
                + Math.abs(LFStart-l1Encoder) + Math.abs(LRStart - l2Encoder)) / 4) / (robot.DRIVE_TICKS_PER_INCH);

        return Math.abs(distanceTraveled);
    }  //close calcDistance

    // distance will power side with highest power
    public void driveArcTurn(double powerLeft, double powerRight, double time) {
        ElapsedTime runTime = new ElapsedTime();
        double currentTime = runTime.time();

        while (((runTime.time() - currentTime) < time) && opMode.opModeIsActive()){

            /*
             * Limit that value of the drive motors so that the power does not exceed 100%
             */
            RFPower = Range.clip(powerRight, -1,1);
            LFPower = Range.clip(powerLeft, -1,1);
            RRPower = Range.clip(powerRight, -1,1);
            LRPower = Range.clip(powerLeft, -1,1);

            /*
             * Apply power to the drive wheels
             */
            setDrivePower(RFPower, RRPower, LFPower, RRPower);

            opMode.idle();

        }   // end of while loop
        motorsHalt();
    }

    public void driveTurn(double targetAngle, double errorFactor) {
        double integral = 0;
        int iterations = 0;
        ElapsedTime timeElapsed = new ElapsedTime();
        double startTime = timeElapsed.time();
        double totalTime;
        double error;
        double Cp = 0.02;
        double Ci = 0.0003;
        double Cd = 0.0001;
        double maxSpeed = 0.6;
        double rotationSpeed;
        double derivative = 0, deltaError, lastError = 0;

        // check to see how far the robot is rotating to decide which gyro sensor value to use
//        if (targetAngle > 90 || targetAngle < -90) {
//            error = gyro360(targetAngle) - targetAngle;
//        } else {
        error = getZAngle() - targetAngle;
//        }

        // nested while loops are used to allow for a final check of an overshoot situation
        while ((Math.abs(error) >= errorFactor) && opMode.opModeIsActive()) {
            while ((Math.abs(error) >= errorFactor) && opMode.opModeIsActive()) {
                deltaError = lastError - error;
                rotationSpeed = ((Cp * error) + (Ci * integral) + (Cd * derivative)) * maxSpeed;

                // Clip motor speed
                rotationSpeed = Range.clip(rotationSpeed, -maxSpeed, maxSpeed);

                if ((rotationSpeed > -0.35) && (rotationSpeed < 0)) {
                    rotationSpeed = -0.35;
                } else if ((rotationSpeed < 0.4) && (rotationSpeed > 0)) {
                    rotationSpeed = 0.4;
                }

                RFPower = rotationSpeed;
                RRPower = rotationSpeed;
                LFPower = -rotationSpeed;
                LRPower = -rotationSpeed;

                setDrivePower(RFPower, RRPower, LRPower, LRPower);

                lastError = error;
                iterations++;

                opMode.telemetry.addData("InitZ/targetAngle value  = ", targetAngle);
                opMode.telemetry.addData("Current Angle  = ", getZAngle());
                opMode.telemetry.addData("Theta/lastError Value= ", lastError);
                opMode.telemetry.addData("CurrentZ/Error Value = ", error);
                opMode.telemetry.addData("zCorrection/derivative Value = ", derivative);

                opMode.telemetry.addData("Right Front = ", RFPower);
                opMode.telemetry.addData("Left Front = ", LFPower);
                opMode.telemetry.addData("Left Rear = ", LRPower);
                opMode.telemetry.addData("Right Rear = ", RRPower);
                opMode.telemetry.update();

                // check to see how far the robot is rotating to decide which gyro sensor value to use
//                if (targetAngle > 90 || targetAngle < -90) {
//                    error = gyro360(targetAngle) - targetAngle;
//                } else {
                error = getZAngle() - targetAngle;
//                }
                opMode.idle();
            }   // end of while Math.abs(error)
            motorsHalt();

            opMode.sleep(10);   // take 10 ms to allow gyro to settle
            // Perform a final calc on the error to confirm that the robot didn't overshoot the
            // target position after the last measurement was taken.
            error = getZAngle() - targetAngle;
            opMode.idle();
        }       // end of outside while loop

        // shut off the drive motors
        motorsHalt();

        totalTime = timeElapsed.time() - startTime;
        opMode.telemetry.addData("Iterations = ", iterations);
        opMode.telemetry.addData("Final Angle = ", getZAngle());
        opMode.telemetry.addData("Total Time Elapsed = ", totalTime);
        opMode.telemetry.update();
    }   //end of the driveTurn Method

    /**
     * Method getZAngle()
     *  -   This method returns the gyro position of the robot.
     * @return zAngle
     */
    public double getZAngle(){
        return (-robot.imu.getAngularOrientation().firstAngle);
    }   // close getZAngle method


    /**
     * Sets power to all four drive motors
     * @param R1 power for right front motor
     * @param L1 power for left front motor
     * @param L2 power for left rear motor
     * @param R2 power for right rear motor
     */
    public void setDrivePower(double R1, double R2, double L1, double L2){
        robot.motorRF.setPower(R1);
        robot.motorRR.setPower(R2);
        robot.motorLF.setPower(L1);
        robot.motorLR.setPower(L2);
    }   // end of the setDrivePower method

    public void driveTime(double power, double time) {
        ElapsedTime runTime = new ElapsedTime();
        double startTime = runTime.time();

        while (((runTime.time() - startTime) < time) && opMode.opModeIsActive()){

            /*
             * Limit that value of the drive motors so that the power does not exceed 100%
             */
            RFPower = Range.clip(power, -1,1);
            LFPower = Range.clip(power, -1,1);
            RRPower = Range.clip(power, -1,1);
            LRPower = Range.clip(power, -1,1);

            /*
             * Apply power to the drive wheels
             */
            setDrivePower(RFPower, RRPower, LFPower, RRPower);

            opMode.idle();

        }   // end of while loop

        motorsHalt();

    } // close driveTime method

    /*
     * Method motorsHalt
     *  -   stops all drive motors
     */
    public void motorsHalt(){
        robot.motorLF.setPower(0);
        robot.motorLR.setPower(0);
        robot.motorRF.setPower(0);
        robot.motorRR.setPower(0);
    }   // end of motorsHalt method

}       // end of DriveClass class
