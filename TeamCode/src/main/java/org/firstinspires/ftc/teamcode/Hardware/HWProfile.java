package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class HWProfile {
    //constants
    public final double clawOpen=0.3;
    public final double clawClosed=0.55;

    public final int liftMax=10000;
    public final int liftMin=0;

    public final double buttonTimeout=0.5;

    /* Public OpMode members. */
    public DcMotor motorLF   = null;
    public DcMotor  motorLR  = null;
    public DcMotor  motorRF     = null;
    public DcMotor  motorRR    = null;
    public DcMotor motorLift = null;
    public BNO055IMU imu = null;
    public Servo servoGrabber = null;


    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private final ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HWProfile(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

//        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)sensorDistance;

        // Define and Initialize Motors
        motorLF = hwMap.get(DcMotorEx.class, "motorLF");
        motorLR = hwMap.get(DcMotorEx.class, "motorLR");
        motorRF = hwMap.get(DcMotorEx.class, "motorRF");
        motorRR = hwMap.get(DcMotorEx.class, "motorRR");
        motorLF.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        motorLR.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        motorRF.setDirection(DcMotor.Direction.REVERSE);
        motorRR.setDirection(DcMotor.Direction.REVERSE);
        motorLF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set all motors to zero power
        motorLF.setPower(0);
        motorLR.setPower(0);
        motorRF.setPower(0);
        motorRR.setPower(0);

        // Set all motors to run without encoders.
        motorLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //lift motor init
        motorLift = hwMap.get(DcMotor.class, "motorLift");
        motorLift.setDirection(DcMotor.Direction.FORWARD);
        motorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //init servos
        servoGrabber = hwMap.get(Servo.class, "servoGrabber");

        //init imu
        imu = hwMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);

    }
}  // end of HWProfile Class
