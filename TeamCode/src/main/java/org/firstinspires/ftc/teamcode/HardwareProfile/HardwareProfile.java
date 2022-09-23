// change comments

/*
12/7/21 added turret continuous servos: turret1 & turret2
        add turretEncoder - encoder added to inTakeMotor referneced via
        turretEncoder object
12/10/21 renamed turret servos
*/
package org.firstinspires.ftc.teamcode.HardwareProfile;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class HardwareProfile {

    /*
     * Constants
     */

    public final double DRIVE_TICKS_PER_INCH = 29.0;

    //drive control constants
    public final double DRIVE_MULTIPLIER=0.5;
    public final double REVERSE_MULTIPLIER=0.75;
    public final double TURN_MULTIPLIER=0.75;

    /*
     * Hardware devices
     */

    //Motors
    public DcMotor motorR1 = null;  // Right Front Drive Motor
    public DcMotor motorL1 = null;  // Left Front Drive Motor
    public DcMotor motorL2 = null;  // Left Rear  Drive Motor
    public DcMotor motorR2 = null;  // Right Rear Drive Motor

    public BNO055IMU imu;       // Internal accelerometer / Gyro sensor

    //Servos
//    public Servo intakeDeployBlue = null; //Right intake deploy servo

    //public distance sensors;
//    public DistanceSensor sensorDistPink=null;

    // public WebcamName webcam = null;

    /* Constructor */
    public HardwareProfile() {

    }   // end of HardwareProfile method

    public void init(HardwareMap hwMap) {

//initialize DcMotor motors

        motorL1 = hwMap.dcMotor.get("motorL1");
        motorL1.setDirection(DcMotor.Direction.REVERSE);
        motorL1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorL1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorL1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorL1.setPower(0);

        motorL2 = hwMap.dcMotor.get("motorL2");
        motorL2.setDirection(DcMotor.Direction.REVERSE);
        motorL2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorL2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorL2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorL2.setPower(0);

        motorR1 = hwMap.dcMotor.get("motorR1");
        motorR1.setDirection(DcMotor.Direction.FORWARD);
        motorR1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorR1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorR1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorR1.setPower(0);

        motorR2 = hwMap.dcMotor.get("motorR2");
        motorR2.setDirection(DcMotor.Direction.FORWARD);
        motorR2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorR2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorR2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorR2.setPower(0);


//initialize servos
//        intakeDeployPink = hwMap.servo.get("intakeDeployPink");


//initialize sensors
//        sensorDistBlue=hwMap.get(DistanceSensor.class, "sensorDistBlue");
//        sensorDistPink=hwMap.get(DistanceSensor.class, "sensorDistPink");

        imu = hwMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);

        /* Webcam device will go here */
        // webcam = hwMap.get(WebcamName.class, "Webcam 1");
    }   // end of init() method

}