/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Hardware.HWProfile;
import org.firstinspires.ftc.teamcode.Libs.DriveClass;

import java.util.List;

/**
 * This 2022-2023 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine which image is being presented to the robot.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Autonomous(name = "Auto: Red Terminal 1 Only", group = "Competition")

public class AutoRedTerminal1Only extends LinearOpMode {

    /*
     * Specify the source for the Tensor Flow Model.
     * If the TensorFlowLite object model is included in the Robot Controller App as an "asset",
     * the OpMode must to load it using loadModelFromAsset().  However, if a team generated model
     * has been downloaded to the Robot Controller's SD FLASH memory, it must to be loaded using loadModelFromFile()
     * Here we assume it's an Asset.    Also see method initTfod() below .
     */
    //private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";
    private static final String TFOD_MODEL_ASSET = "model_20221208_140441.tflite";
    // private static final String TFOD_MODEL_FILE  = "/sdcard/FIRST/tflitemodels/CustomTeamModel.tflite";


/*    private static final String[] LABELS = {
            "1 Bolt",
            "2 Bulb",
            "3 Panel"
    };
  */
    private static final String[] LABELS = {
            "Logo",
            "Peacock",
            "Qrcode"
    };

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "AfHl2GP/////AAABmeJc93xOhk1MvZeKbP5E43taYJ6kodzkhsk5wOLGwZI3wxf7v1iTx2Mem/VZSEtpxb3U2fMO7n0EUxSeHRWhOXeX16dMFcjfalezjo3ZkzBuG/y2r4kgLwKs4APyAIClBAon+tf/W/4NkTkYuHGo8zZ0slH/iBpqxvblpNURsG5h4VxPFgF5D/FIfmjnddzQpa4cGarle/Zvuah6q2orUswun31P6ZLuIJvdOIQf7o/ruoRygsSXfVYc35w+Xwm+bwjpZUNzHHYvRNrp0HNWC3Fr2hd0TqWKIIYlCoHj0m5OKX22Ris23V8PdKM/i4/ZIy8JewJXetv1rERC5bfHmUXCS4Rl7RjR+ZscQ5aA0nr8";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;
    double position = 3;
    private final static HWProfile robot = new HWProfile();
    private LinearOpMode opMode = this;

    DriveClass drive = new DriveClass(robot, opMode);

    @Override

    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();
        /*
         * Setup the initial state of the robot
         */

        State autoState = State.DETECT_CONE;

        robot.init(hardwareMap);


        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can increase the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1.0, 16.0/9.0);
        }

        /** Wait for the game to begin */


        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        while(!opModeIsActive()) {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Objects Detected", updatedRecognitions.size());

                    // step through the list of recognitions and display image position/size information for each one
                    // Note: "Image number" refers to the randomized image orientation/number
                    for (Recognition recognition : updatedRecognitions) {
                        double col = (recognition.getLeft() + recognition.getRight()) / 2 ;
                        double row = (recognition.getTop()  + recognition.getBottom()) / 2 ;
                        double width  = Math.abs(recognition.getRight() - recognition.getLeft()) ;
                        double height = Math.abs(recognition.getTop()  - recognition.getBottom()) ;

                        telemetry.addData(""," ");
                        telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100 );
                        telemetry.addData("- Position (Row/Col)","%.0f / %.0f", row, col);
                        telemetry.addData("- Size (Width/Height)","%.0f / %.0f", width, height);
             /*           if(recognition.getLabel() == "1 Bolt"){
                            position =1;
                        } else if(recognition.getLabel() == "2 Bulb" ){
                            position = 2;
                        } else position = 3;
               */
                        // make Qrcode the default, least reliable
                        if(recognition.getLabel() == "Peacock"){
                            position =1;
                        } else if(recognition.getLabel() == "Logo" ){
                            position = 3;
                        } else position = 2;
                    }
                    telemetry.update();
                }
            }

        }  // end of while

        waitForStart();

        while(opModeIsActive()){

            switch (autoState) {
                case TEST:

                    break;

                case DETECT_CONE:
                    // close grab the cone before starting
                    drive.closeClaw();
                    sleep(400);

                    autoState = State.SCORE_LOW;
                    break;

                case SCORE_LOW:
                    drive.liftLowerJunction();
                    drive.driveDistance(0.3, 90, 11);

                    drive.PIDRotate(0, 2);

                    // drove towards the junction
                    drive.driveDistance(0.3, 0, 5);

//                    sleep(500);
                    drive.liftReset();

                    sleep(400);
                    drive.openClaw();

                    // drove towards the junction
                    drive.driveDistance(0.3, 180, 4);

                    drive.driveDistance(0.3, -90,11);

//                    drive.PIDRotate(0, 2);

                    // push signal cone out of the way
                    drive.driveDistance(0.3, 0, 76);

                    // back up to turn towards cone stack
                    drive.driveDistance(0.3, 180, 40);

                    autoState = State.PARK;
                    break;

                case RETRIEVE_CONE2:
                    //                  drive.PIDRotate(0, 2);

                    // push signal cone out of the way
                    drive.driveDistance(0.3, 0, 76);

                    // back up to turn towards cone stack
                    drive.driveDistance(0.3, 180, 4.5);

                    // raise lift to appropriate height
                    drive.liftPosition(1150);

                    // turn towards cone stack
                    drive.PIDRotate(-90, 2);
                    drive.driveDistance(0.4, 0, 12);

                    // realign the robot
                    drive.PIDRotate(-90, 2);

                    // drive towards cones
                    drive.driveDistance(0.4, 0, 13);

                    drive.closeClaw();
                    sleep(500);

                    drive.liftLowerJunction();
                    sleep(500);

                    drive.driveDistance(0.4, 180, 25);

                    autoState = State.SCORE_JUNCTION2;
                    break;

                case SCORE_JUNCTION2:
                    drive.PIDRotate(-90, 2);

                    // strafe towards junction 2
                    drive.driveDistance(0.3, -90, 11);

                    // drive forward to place the cone
//                    drive.driveDistance(0.3, 0, 4);

                    // lower the lift
                    drive.liftPosition(910);
                    sleep(400);

                    // open claw and drive backwards
                    drive.openClaw();
                    drive.driveDistance(0.3, 180, 2);

                    // strafe back to starting position
                    drive.driveDistance(0.3, 90, 11);

                    autoState = State.RETRIEVE_CONE3;
                    break;

                case RETRIEVE_CONE3:
                    // align robot towards the cones
                    drive.PIDRotate(-90, 2);

                    // drive towards the cone stack to grab another
                    drive.driveDistance(0.3, 0, 32);

                    // close the claw around the cone
                    drive.closeClaw();
                    sleep(500);

                    // raise the lift
                    drive.liftHighJunction();
                    sleep(300);

                    autoState = State.SCORE_CONE3;

                    break;

                case SCORE_CONE3:
                    // drive back towards the high goal
                    drive.driveDistance(0.3, 180, 41.5);

                    // turn towards the high junction
                    drive.PIDRotate(0, 2);

                    sleep(500);

                    // rotate towards the high goal
                    drive.PIDRotate(0, 2);

                    // position in scoring position
                    drive.driveDistance(0.3, 0, 0.5);
//                    drive.driveDistance(0.3, 0, 3);

                    // lower the lift to place the cone
                    drive.liftPosition(600);
                    sleep(1000);

                    // open the claw to release the cone
                    drive.openClaw();

                    // raise the lift to avoid entangling with the junction
                    drive.liftHighJunction();
                    sleep(1000);

                    // back away from the junction to allow room for turning
                    drive.driveDistance(0.2, 180, 3);

                    // turn towards the cone stack to retrieve another cone or park
                    drive.PIDRotate(-90, 2);
                    drive.liftPosition(600);
                    sleep(500);

                    autoState = State.PARK;
                    break;

                case PARK:

                    if(position == 1) {
                        // reset the lift
                        drive.liftReset();
                        drive.openClaw();

                        // rotate towards the park 1 position
                        //drive.PIDRotate(-90, 2);

                        // drive to park position 1
                        drive.driveDistance(0.3, -90,30);

                        // rotate into position for field centric drive
                        drive.PIDRotate(0,2);
                    } else if (position == 2) {
                        // reset the lift
                        drive.liftReset();
                        drive.openClaw();

                        // rotate towards the outside wall position
                        /*drive.PIDRotate(-90, 2);

                        // drive to park position 1
         //               drive.driveDistance(0.3, 0,9);
                        */

                        // rotate into position for field centric drive
                        drive.PIDRotate(0,2);
                    } else {
                        // reset the lift
                        drive.liftReset();
                        drive.openClaw();

                        // rotate towards the outside wall position
                        //drive.PIDRotate(90, 2);

                        // drive to park position 1
                        drive.driveDistance(0.3, 90,24);

                        // rotate into position for field centric drive
                        drive.PIDRotate(0,2);
                    }

                    while(opModeIsActive() && robot.motorLift.getCurrentPosition() > 10){
                        drive.liftReset();
                    }

                    autoState = State.HALT;

                    break;

                case HALT:

                    // Stop all motors
                    drive.motorsHalt();
                    robot.motorLift.setPower(0);
                    drive.openClaw();

                    // End the program
                    requestOpModeStop();

                    break;
            }   // end of the switch state


        } // end of while(opModeIsActive())

    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    enum State {
        TEST, DETECT_CONE, SCORE_LOW, RETRIEVE_CONE2, SCORE_JUNCTION2, RETRIEVE_CONE3, SCORE_CONE3, SCORE_CORNER, PARK, PARK2, HALT;
    }   // end of enum State

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.6f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        // tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
    }
}