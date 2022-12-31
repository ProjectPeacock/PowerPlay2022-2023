package org.firstinspires.ftc.teamcode.Libs;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.HWProfile;

public class MechanismClass {

    private HWProfile robot;
    public double RF, LF, LR, RR;
    public LinearOpMode opMode;
    ElapsedTime runTime = new ElapsedTime();

    /*
     * Constructor method
     */
    public MechanismClass(HWProfile myRobot, LinearOpMode myOpMode){
        robot = myRobot;
        opMode = myOpMode;

    }   // close RobotClass constructor Method

    /*
     * ###########################################################################################
     * ###########################################################################################
     * #######################      ARM & LIFT CONTROLS      #####################################
     * ###########################################################################################
     * ###########################################################################################
     */

    /**
     * Method: liftPosition
     *  -   raise the lift to the desired position
     * @param position
     */
    public void liftPosition(int position, double power){
        robot.motorLiftFront.setTargetPosition(position);
        robot.motorLiftRear.setTargetPosition(position);
        robot.motorLiftFront.setPower(power);
        robot.motorLiftRear.setPower(power);
    }

    /**
     * Method: openClaw
     *  -   open the claw
     */
    public void openClaw(){
        robot.servoGrabber.setPosition(robot.CLAW_OPEN);
    }

    /**
     * Method: closeClaw
     *  -   close the claw
     */
    public void closeClaw(){
        robot.servoGrabber.setPosition(robot.CLAW_CLOSE);
    }



}   // close the driveMecanum class