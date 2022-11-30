package org.firstinspires.ftc.teamcode.OpModes;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Hardware.HWProfile;

@TeleOp(name = "Teleop Mode", group = "Competition")

public class MecanumTeleOp extends LinearOpMode {
    private final static HWProfile robot = new HWProfile();

    @Override
    public void runOpMode() {
        double v1, v2, v3, v4, robotAngle;
        double theta;
        double theta2 = 180;
        double r;
        double power = robot.MAX_DRIVE_POWER;
        double rightX, rightY;
        boolean fieldCentric = true;
        int liftPosition = 0;

        ElapsedTime currentTime = new ElapsedTime();
        double buttonPress = currentTime.time();

        robot.init(hardwareMap);
        robot.motorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        telemetry.addData("Ready to Run: ", "GOOD LUCK");
        telemetry.update();

        boolean shippingElement = true;
        boolean armDeployed = true;

        boolean clawOpen = true;

        waitForStart();

        while (opModeIsActive()) {

            // give drive the ability to adjust the max speed of the robot
            if(gamepad1.right_trigger>0.1&&gamepad1.right_trigger<0.8) {
                power*=0.5;
            }else if(gamepad1.right_trigger<0.1){
                power=robot.MAX_DRIVE_POWER;
            }

            /*******************************************
             ****** Mecanum Drive Control section ******
             *******************************************/
            if (fieldCentric) {             // verify that the user hasn't disabled field centric drive
                theta = robot.imu.getAngularOrientation().firstAngle - 180;
            } else {
                theta = 0;      // do not adjust for the angular position of the robot
            }

            robotAngle = Math.atan2(gamepad1.left_stick_y, (-gamepad1.left_stick_x)) - Math.PI / 4;
            rightX = gamepad1.right_stick_x;
            rightY = gamepad1.right_stick_y;
            r = -Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);

            v1 = (r * Math.cos(robotAngle - Math.toRadians(theta + theta2)) + rightX + rightY);
            v2 = (r * Math.sin(robotAngle - Math.toRadians(theta + theta2)) - rightX + rightY);
            v3 = (r * Math.sin(robotAngle - Math.toRadians(theta + theta2)) + rightX + rightY);
            v4 = (r * Math.cos(robotAngle - Math.toRadians(theta + theta2)) - rightX + rightY);

            robot.motorLF.setPower(com.qualcomm.robotcore.util.Range.clip((v1), -power, power));
            robot.motorRF.setPower(com.qualcomm.robotcore.util.Range.clip((v2), -power, power));
            robot.motorLR.setPower(com.qualcomm.robotcore.util.Range.clip((v3), -power, power));
            robot.motorRR.setPower(com.qualcomm.robotcore.util.Range.clip((v4), -power, power));

            // Control which direction is forward and which is backward from the driver POV
            if (gamepad1.y && (currentTime.time() - buttonPress) > robot.BUTTON_TIMEOUT) {
                if (theta2 == 180) {
                    theta2 = 0;
                } else {
                    theta2 = 180;
                }
                buttonPress = currentTime.time();
            }   // end if (gamepad1.y && ...)


            /*
             * #############################################################
             * #################### CLAW CONTROL ###########################
             * #############################################################
             */
            if(gamepad1.a&&(currentTime.time() - buttonPress) > robot.BUTTON_TIMEOUT){
                clawOpen=!clawOpen;
                buttonPress = currentTime.time();
            }

            if (clawOpen) {
                robot.servoGrabber.setPosition(robot.CLAW_OPEN);
            } else {
                robot.servoGrabber.setPosition(robot.CLAW_CLOSE);
            }

            /*
             * #############################################################
             * #################### LIFT CONTROL ###########################
             * #############################################################
             */
            if (gamepad2.right_trigger > 0.1 && robot.motorLift.getCurrentPosition()<=robot.MAX_LIFT_VALUE) {
//                liftPosition = liftPosition + 1;
                robot.motorLift.setPower(gamepad2.right_trigger);

            } else if (gamepad2.left_trigger > 0.1 && robot.motorLift.getCurrentPosition() >= robot.MIN_LIFT_VALUE) {
                robot.motorLift.setPower(-gamepad2.left_trigger);
//                liftPosition = liftPosition - 1;
            } else robot.motorLift.setPower(0);

            // limit the values of liftPosition => This shouldn't be necessary if logic above works
//            Range.clip(liftPosition, robot.MIN_LIFT_VALUE, robot.MAX_LIFT_VALUE);

            // move lift to target position
//            robot.motorLift.setTargetPosition(liftPosition);
//            robot.motorLift.setPower(1);

        }   // end of while(opModeIsActive)
    }   // end of runOpMode()
}       // end of MSTeleop class