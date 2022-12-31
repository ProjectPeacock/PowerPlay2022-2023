package org.firstinspires.ftc.teamcode.OpModes;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Hardware.HWProfile;
import org.firstinspires.ftc.teamcode.Libs.AutoClass;

import java.util.List;

@Config
@TeleOp(name = "Broken Bot", group = "Development")

public class BrokenBot extends LinearOpMode {
    private final static HWProfile robot = new HWProfile();
    FtcDashboard dashboard;
    public static double l1_CLAW_OPEN = robot.CLAW_OPEN;
    public static double l2_CLAW_CLOSE = robot.CLAW_CLOSE;
    public static int l3_LIFT_JUNCTION_HIGH = robot.LIFT_JUNCTION_HIGH;
    public static int l4_LIFT_JUNCTION_MID = robot.LIFT_JUNCTION_MID;
    public static int l5_LIFT_JUNCTION_LOW = robot.LIFT_JUNCTION_LOWER;
    public static int l6_LIFT_POSITION = 0;
    public static double l7_Lift_Up_Power = robot.LIFT_UP_POWER;
    public static double l8_Lift_Down_Power = robot.LIFT_DOWN_POWER;

    @Override
    public void runOpMode() {
        boolean fieldCentric = false;
        boolean liftToPosition = true;
        int liftPosition = 0;
        LinearOpMode opMode = this;

        dashboard = FtcDashboard.getInstance();
        TelemetryPacket dashTelemetry = new TelemetryPacket();

        robot.init(hardwareMap);
        GamepadEx gp1 = new GamepadEx(gamepad1);
        GamepadEx gp2 = new GamepadEx(gamepad2);
        ButtonReader clawToggleButton = new ButtonReader(gp1, GamepadKeys.Button.RIGHT_BUMPER);

        AutoClass drive = new AutoClass(robot, opMode);

        telemetry.addData("Ready to Run: ", "GOOD LUCK");
        telemetry.update();


        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        double liftPower=0;
        boolean clawToggle=false, clawReady=false;

        // post telemetry to FTC Dashboard as well
        dashTelemetry.put("01 - IMU Angle X = ", robot.imu.getAngles()[0]);
        dashTelemetry.put("02 - IMU Angle Y = ", robot.imu.getAngles()[1]);
        dashTelemetry.put("03 - IMU Angle Z = ", robot.imu.getAngles()[2]);
        dashTelemetry.put("04 - Lift Front Encoder Value = ", robot.motorLiftFront.getCurrentPosition());
        dashTelemetry.put("05 - Lift Rear Encoder Value = ", robot.motorLiftRear.getCurrentPosition());
        dashTelemetry.put("06 - Claw Value = ", robot.servoGrabber.getPosition());
        dashTelemetry.put("07 - GP1.Button.A = ", "RESET LIFT");
        dashTelemetry.put("08 - GP1.Button.B = ", "LIFT LOW JUNCTION");
        dashTelemetry.put("09 - GP1.Button.X = ", "LIFT MID JUNCTION");
        dashTelemetry.put("10 - GP1.Button.Y = ", "LIFT HIGH JUNCTION");
        dashTelemetry.put("11 - GP2.Button.A = ", "Custom Position - program stack cone levels");
        dashTelemetry.put("12 - Lift Power = ", liftPower);
        dashboard.sendTelemetryPacket(dashTelemetry);

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad2.right_trigger>0.1) {
                robot.autoLight.setPower(-gamepad2.right_trigger);
            }else{
                robot.autoLight.setPower(0);
            }
            if(fieldCentric){
                robot.mecanum.driveFieldCentric(gp1.getLeftX(),gp1.getLeftY(),-gp1.getRightX()*robot.TURN_MULTIPLIER,robot.imu.getRotation2d().getDegrees()+180, false);
            }else{
                robot.mecanum.driveRobotCentric(gp1.getLeftX(),gp1.getLeftY(),-gp1.getRightX()*robot.TURN_MULTIPLIER, false);
            }

            if(clawToggleButton.isDown()&&clawReady){
                clawToggle=!clawToggle;
            }
            if(!clawToggleButton.isDown()){
                clawReady=true;
            }else{
                clawReady=false;
            }
            if (clawToggle) {
                robot.servoGrabber.setPosition(l1_CLAW_OPEN);
            } else {
                robot.servoGrabber.setPosition(l2_CLAW_CLOSE);
            }


            if(gp1.isDown(GamepadKeys.Button.DPAD_UP)) {
                robot.motorLF.set(1);
            } else if (gp1.isDown(GamepadKeys.Button.DPAD_DOWN)){
                robot.motorLR.set(1);
            } else if (gp1.isDown(GamepadKeys.Button.DPAD_LEFT)) {
                robot.motorRF.set(1);
            } else if (gp1.isDown(GamepadKeys.Button.DPAD_RIGHT)){
                robot.motorRR.set(1);
            }

            /*
             * #############################################################
             * #################### LIFT CONTROL ###########################
             * #############################################################
             */

            if (gp1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.1) {
                liftPosition = liftPosition - 10;
                liftPower = l8_Lift_Down_Power;

            } else if (gp1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.1) {
                liftPosition = liftPosition + 10;
                liftPower = l7_Lift_Up_Power;
            } else {
            }

            if(gp1.isDown(GamepadKeys.Button.A)){
                liftPosition = robot.LIFT_RESET;
                liftPower = l8_Lift_Down_Power;
            }

            if(gp1.isDown(GamepadKeys.Button.B)){
                liftPosition = l5_LIFT_JUNCTION_LOW;
                liftPower = l7_Lift_Up_Power;
            }

            if(gp1.isDown(GamepadKeys.Button.X)){
                liftPosition = l4_LIFT_JUNCTION_MID;
                liftPower = l7_Lift_Up_Power;
            }

            if(gp1.isDown(GamepadKeys.Button.Y)){
                liftPosition =l3_LIFT_JUNCTION_HIGH;
                liftPower = l7_Lift_Up_Power;
            }

            if(gp2.isDown(GamepadKeys.Button.A)){
                liftPosition = l6_LIFT_POSITION;
                liftPower = l7_Lift_Up_Power;
            }

            liftPosition = Range.clip(liftPosition, robot.LIFT_RESET, robot.MAX_LIFT_VALUE);

            robot.motorLiftFront.setTargetPosition(liftPosition);
            robot.motorLiftRear.setTargetPosition(liftPosition);
            robot.motorLiftFront.setPower(liftPower);
            robot.motorLiftRear.setPower(liftPower);

            // Provide user feedback
            telemetry.addData("A:", "Lift Reset");
            telemetry.addData("B:", "Lift Low");
            telemetry.addData("X:", "Lift Mid");
            telemetry.addData("Y:", "Lift High");
            telemetry.addData("Lift Front Encoder Value = ", robot.motorLiftFront.getCurrentPosition());
            telemetry.addData("Lift Rear Encoder Value = ", robot.motorLiftRear.getCurrentPosition());
            telemetry.addData("IMU Angle X = ", robot.imu.getAngles()[0]);
            telemetry.addData("IMU Angle Y = ", robot.imu.getAngles()[1]);
            telemetry.addData("IMU Angle Z = ", robot.imu.getAngles()[2]);
            telemetry.addData("Left Stick X = ", gp1.getLeftX());
            telemetry.addData("Left Stick Y = ", gp1.getLeftY());
            telemetry.addData("Right Stick X = ", gp1.getRightX());
            telemetry.addData("Right Stick Y = ", gp1.getRightY());
            telemetry.update();

            // post telemetry to FTC Dashboard as well
            dashTelemetry.put("01 - IMU Angle X = ", robot.imu.getAngles()[0]);
            dashTelemetry.put("02 - IMU Angle Y = ", robot.imu.getAngles()[1]);
            dashTelemetry.put("03 - IMU Angle Z = ", robot.imu.getAngles()[2]);
            dashTelemetry.put("04 - Lift Front Encoder Value = ", robot.motorLiftFront.getCurrentPosition());
            dashTelemetry.put("05 - Lift Rear Encoder Value = ", robot.motorLiftRear.getCurrentPosition());
            dashTelemetry.put("06 - Claw Value = ", robot.servoGrabber.getPosition());
            dashTelemetry.put("07 - GP1.Button.A = ", "RESET LIFT");
            dashTelemetry.put("08 - GP1.Button.B = ", "LIFT LOW JUNCTION");
            dashTelemetry.put("09 - GP1.Button.X = ", "LIFT MID JUNCTION");
            dashTelemetry.put("10 - GP1.Button.Y = ", "LIFT HIGH JUNCTION");
            dashTelemetry.put("11 - GP2.Button.A = ", "Custom Position - program stack cone levels");
            dashTelemetry.put("12 - Lift Power = ", liftPower);
            dashboard.sendTelemetryPacket(dashTelemetry);

        }   // end of while(opModeIsActive)
    }   // end of runOpMode()

}       // end of BrokenBot class