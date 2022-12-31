package org.firstinspires.ftc.teamcode.OpModes;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Lift Test", group = "Development")
@Disabled
public class LiftTest extends LinearOpMode {
    public DcMotorEx motorLiftFront = null;
    public DcMotorEx motorLiftRear = null;

    @Override
    public void runOpMode() {

        //lift motor init
        motorLiftFront = hardwareMap.get(DcMotorEx.class, "motorLiftFront");
        motorLiftFront.setDirection(DcMotorEx.Direction.FORWARD);
        motorLiftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorLiftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorLiftFront.setTargetPosition(0);
        motorLiftFront.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        motorLiftFront.setPower(0);

        motorLiftRear = hardwareMap.get(DcMotorEx.class, "motorLiftRear");
        motorLiftRear.setDirection(DcMotorEx.Direction.FORWARD);
//        motorLiftRear.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorLiftRear.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorLiftRear.setTargetPosition(0);
        motorLiftRear.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        motorLiftRear.setPower(0);


        telemetry.addData("Ready to Run: ", "GOOD LUCK");
        telemetry.update();


        waitForStart();

        while (opModeIsActive()) {

            if(gamepad2.a){
                motorLiftFront.setTargetPosition(0);
                motorLiftFront.setPower(0.3);
            }

            if(gamepad2.b){
                motorLiftFront.setTargetPosition(100);
                motorLiftFront.setPower(0.3);
            }

            if(gamepad2.x){
                motorLiftRear.setTargetPosition(0);
                motorLiftRear.setPower(0.3);
            }

            if(gamepad2.y){
                motorLiftRear.setTargetPosition(100);
                motorLiftRear.setPower(1);
            }

            // Provide user feedback
            telemetry.addData("Motor Lift Front:", motorLiftFront.getCurrentPosition());
            telemetry.addData("Motor Lift Rear:", motorLiftRear.getCurrentPosition());
            telemetry.update();

        }   // end of while(opModeIsActive)
    }   // end of runOpMode()
}       // end of LiftTest class