package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Hardware.HWProfile;
import org.firstinspires.ftc.teamcode.Libs.MechanismClass;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.List;

@Autonomous(name = "RR Auto: Red Terminal", group = "RoadRunner")

public class AutoRedTerminal extends LinearOpMode {

    // set up the dashboard
    FtcDashboard dashboard;

    // set up Vuforia amd Tensorflow for signal sleeve sensing
    private static final String VUFORIA_KEY =
            "AfHl2GP/////AAABmeJc93xOhk1MvZeKbP5E43taYJ6kodzkhsk5wOLGwZI3wxf7v1iTx2Mem/VZSEtpxb3U2fMO7n0EUxSeHRWhOXeX16dMFcjfalezjo3ZkzBuG/y2r4kgLwKs4APyAIClBAon+tf/W/4NkTkYuHGo8zZ0slH/iBpqxvblpNURsG5h4VxPFgF5D/FIfmjnddzQpa4cGarle/Zvuah6q2orUswun31P6ZLuIJvdOIQf7o/ruoRygsSXfVYc35w+Xwm+bwjpZUNzHHYvRNrp0HNWC3Fr2hd0TqWKIIYlCoHj0m5OKX22Ris23V8PdKM/i4/ZIy8JewJXetv1rERC5bfHmUXCS4Rl7RjR+ZscQ5aA0nr8";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";

    //private static final String TFOD_MODEL_ASSET = "model_20221208_140441.tflite";
//     private static final String TFOD_MODEL_ASSET  = "/sdcard/FIRST/tflitemodels/CustomTeamModel.tflite";  // default model - save for emergency
    private static final String[] LABELS = {
            "Logo",
            "Peacock",
            "Qrcode"
    };

    // set up the hardware and opmode configurations
    private final static HWProfile robot = new HWProfile();
    private LinearOpMode opMode = this;
    MechanismClass mechs = new MechanismClass(robot, opMode);

    // define local variables
    double parkPosition = 3;        // set the default parking position

    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the Vuforia Localizer, so we create that
        // first.
        initVuforia();
        initTfod();
        /*
         * Setup the initial state of the robot
         */

        robot.init(hardwareMap);
        dashboard = FtcDashboard.getInstance();
        FtcDashboard.getInstance().startCameraStream(vuforia, 0);

        State autoState = State.DETECT_CONE;
        ElapsedTime currentTime = new ElapsedTime();
        double startTime = 0;

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

        /* ROADRUNNER settings */
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // We want to start the bot at x: 10, y: -8, heading: 90 degrees
        Pose2d startPose = new Pose2d(-36, -66, Math.toRadians(0));
        drive.setPoseEstimate(startPose);

        // trajScoreLowJunction1 => navigates the bot to the first low junction to score
        Trajectory trajScoreLowJunction1 = drive.trajectoryBuilder(new Pose2d(-66, -36, Math.toRadians(0)))
                .splineTo(new Vector2d(-30,-60), Math.toRadians(45))
                .build();

        // trajRetrieveCone5 => navigates the bot from the first junction to the cone stack to grab the top cone
        Trajectory trajRetrieveCone5 = drive.trajectoryBuilder(trajScoreLowJunction1.end())
                .splineTo(new Vector2d(-36,-12), Math.toRadians(-90))
                .splineTo(new Vector2d(-66, -12), Math.toRadians(-90))
                .build();

        // trajScoreLowJunction2 => navigates the bot from the cone stack to the 2nd low junction
        Trajectory trajScoreLowJunction2 = drive.trajectoryBuilder(trajRetrieveCone5.end())
                .splineTo(new Vector2d(-30,-12), Math.toRadians(-135))
                .splineTo(new Vector2d(-36, -18), Math.toRadians(-135))
                .build();

        // trajRetrieveCone4 => navigates the bot from the 2nd low junction to the cone stack to grab cone 4
        Trajectory trajRetrieveCone4 = drive.trajectoryBuilder(trajScoreLowJunction2.end())
                .splineTo(new Vector2d(-30,-12), Math.toRadians(-90))
                .splineTo(new Vector2d(-66, -12), Math.toRadians(-90))
                .build();

        // trajScoreMidJunction1 => navigates the bot from the cone stack to the mid junction
        Trajectory trajScoreMidJunction1 = drive.trajectoryBuilder(trajRetrieveCone4.end())
                .splineTo(new Vector2d(-30,-12), Math.toRadians(-225))
                .splineTo(new Vector2d(-24, -18), Math.toRadians(-225))
                .build();

        // trajRetrieveCone3 => navigates the bot from the mid junction to the cone stack to grab cone 3
        Trajectory trajRetrieveCone3 = drive.trajectoryBuilder(trajScoreMidJunction1.end())
                .splineTo(new Vector2d(-30,-12), Math.toRadians(-90))
                .splineTo(new Vector2d(-66, -12), Math.toRadians(-90))
                .build();

        // trajScoreHighJunction1 => navigates the bot from the cone stack to the high junction
        Trajectory trajScoreHighJunction1 = drive.trajectoryBuilder(trajRetrieveCone3.end())
                .splineTo(new Vector2d(-12,-12), Math.toRadians(-45))
                .splineTo(new Vector2d(-18, -6), Math.toRadians(-45))
                .build();

        // trajRetrieveCone2 => navigates the bot from the 1st high junction to the cone stack
        Trajectory trajRetrieveCone2 = drive.trajectoryBuilder(trajScoreHighJunction1.end())
                .splineTo(new Vector2d(-12,-12), Math.toRadians(-90))
                .splineTo(new Vector2d(-66, -12), Math.toRadians(-90))
                .build();

        // trajScoreHighJunction2 => navigates the bot from the cone stack to the 2nd high junction
        Trajectory trajScoreHighJunction2 = drive.trajectoryBuilder(trajRetrieveCone2.end())
                .splineTo(new Vector2d(-12,-12), Math.toRadians(-225))
                .splineTo(new Vector2d(-18, -6), Math.toRadians(-225))
                .build();

        // trajRetrieveCone1 => navigates the bot from the 2nd high junction to the cone stack to grab cone 1
        Trajectory trajRetrieveCone1 = drive.trajectoryBuilder(trajScoreHighJunction2.end())
                .splineTo(new Vector2d(-66,-12), Math.toRadians(-90))
                .build();

        // trajScoreHighJunction3 => navigates the bot from the cone stack to the 2nd high junction
        Trajectory trajScoreHighJunction3 = drive.trajectoryBuilder(trajRetrieveCone2.end())
                .splineTo(new Vector2d(-12,-12), Math.toRadians(-225))
                .splineTo(new Vector2d(-18, -6), Math.toRadians(-225))
                .build();

        // trajPark1 => navigates the bot to Park Zone 1
        Trajectory trajPark1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .splineTo(new Vector2d(-66,-12), Math.toRadians(-0))
                .build();

        // trajPark2 => navigates the bot to Park Zone 2
        Trajectory trajPark2 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .splineTo(new Vector2d(-36,-12), Math.toRadians(-0))
                .build();

        // trajPark3 => navigates the bot to Park Zone 3
        Trajectory trajPark3 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .splineTo(new Vector2d(-12,-12), Math.toRadians(-0))
                .build();

        /** Wait for the game to begin which determining signal sleeve parking zone */

        // turn on the autoLight to get a better view of the signal sleeve
        robot.autoLight.setPower(1);
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
                        if(recognition.getLabel() == "Peacock"){
                            parkPosition =1;
                        } else if(recognition.getLabel() == "Logo" ){
                            parkPosition = 3;
                        } else parkPosition = 2;
                    }
                    telemetry.update();
                }
            }
        }  // end of while

//        waitForStart();

        // set the autonomous start time
        startTime = currentTime.time();

        while(opModeIsActive()){

            switch (autoState) {
                case TEST:

                    break;

                case DETECT_CONE:
                    // turn off the signal cone light
                    robot.autoLight.setPower(0);

                    // close grab the cone before starting
                    mechs.closeClaw();
                    sleep(400);

                    if(!opModeIsActive()) {
                        autoState = State.HALT;
                    } else if(currentTime.time() - startTime  > robot.PARK_TIME) {
                        autoState = State.PARK;
                    } else {
                        autoState = State.SCORE_LOW;
                    }
                    break;

                case SCORE_LOW:
                    if(opModeIsActive()) mechs.liftPosition(robot.LIFT_JUNCTION_LOWER, robot.LIFT_UP_POWER);

                    // Drive forward and rotate towards low junction
                    if(opModeIsActive()) drive.followTrajectory(trajScoreLowJunction1);

                    //lower the lift to place the cone
                    if(opModeIsActive()) mechs.liftPosition(robot.LIFT_CONE5, robot.LIFT_DOWN_POWER);
                    sleep(200);

                    // open the claw
                    if(opModeIsActive()) mechs.openClaw();

                    if(!opModeIsActive()) {
                        autoState = State.HALT;
                    } else if(currentTime.time() - startTime  > robot.PARK_TIME) {
                        autoState = State.PARK;
                    } else {
                        autoState = State.RETRIEVE_CONE5;
                    }
                    break;

                case RETRIEVE_CONE5:
                    // reset the lift position
                    if(opModeIsActive()) mechs.liftPosition(robot.LIFT_CONE5, robot.LIFT_UP_POWER);

                    // drive to the cone stack
                    if(opModeIsActive()) drive.followTrajectory(trajRetrieveCone5);

                    // grab the cone and lift
                    if(opModeIsActive()) mechs.closeClaw();
                    sleep(300);
                    if(opModeIsActive()) mechs.liftPosition(robot.LIFT_JUNCTION_LOWER, robot.LIFT_UP_POWER);
                    sleep(200);

                    if(!opModeIsActive()) {
                        autoState = State.HALT;
                    } else if(currentTime.time() - startTime  > robot.PARK_TIME) {
                        autoState = State.PARK;
                    } else {
                        autoState = State.SCORE_LOW_JUNCTION2;
                    }
                    break;

                case SCORE_LOW_JUNCTION2:
                    // drive to the 2nd low junction
                    if(opModeIsActive()) drive.followTrajectory(trajScoreLowJunction2);

                    // lower the lift to score the cone and release the cone
                    if(opModeIsActive()) mechs.liftPosition(robot.LIFT_CONE4, robot.LIFT_DOWN_POWER);
                    sleep(300);
                    if(opModeIsActive()) mechs.openClaw();
                    sleep(200);

                    if(!opModeIsActive()) {
                        autoState = State.HALT;
                    } else if(currentTime.time() - startTime  > robot.PARK_TIME) {
                        autoState = State.PARK;
                    } else {
                        autoState = State.RETRIEVE_CONE4;
                    }
                    break;

                case RETRIEVE_CONE4:
                    // reset the lift position
                    if(opModeIsActive()) mechs.liftPosition(robot.LIFT_CONE4, robot.LIFT_UP_POWER);

                    // drive to the cone stack
                    drive.followTrajectory(trajRetrieveCone4);

                    // grab the cone and lift
                    if(opModeIsActive()) mechs.closeClaw();
                    sleep(300);
                    if(opModeIsActive()) mechs.liftPosition(robot.LIFT_JUNCTION_MID, robot.LIFT_UP_POWER);
                    sleep(200);

                    if(!opModeIsActive()) {
                        autoState = State.HALT;
                    } else if(currentTime.time() - startTime  > robot.PARK_TIME) {
                        autoState = State.PARK;
                    } else {
                        autoState = State.SCORE_MID1;
                    }
                    break;

                case SCORE_MID1:
                    // drive to the 2nd low junction
                    if(opModeIsActive()) drive.followTrajectory(trajScoreMidJunction1);

                    // lower the lift to score the cone and release the cone
                    if(opModeIsActive()) mechs.liftPosition(robot.LIFT_CONE3, robot.LIFT_DOWN_POWER);
                    sleep(300);
                    if(opModeIsActive()) mechs.openClaw();
                    sleep(200);

                    if(!opModeIsActive()) {
                        autoState = State.HALT;
                    } else if(currentTime.time() - startTime  > robot.PARK_TIME) {
                        autoState = State.PARK;
                    } else {
                        autoState = State.RETRIEVE_CONE3;
                    }
                    break;

                case RETRIEVE_CONE3:
                    // reset the lift position
                    if(opModeIsActive()) mechs.liftPosition(robot.LIFT_CONE3, robot.LIFT_UP_POWER);

                    // drive to the cone stack
                    drive.followTrajectory(trajRetrieveCone3);

                    // grab the cone and lift
                    if(opModeIsActive()) mechs.closeClaw();
                    sleep(300);
                    if(opModeIsActive()) mechs.liftPosition(robot.LIFT_JUNCTION_HIGH, robot.LIFT_UP_POWER);
                    sleep(200);

                    if(!opModeIsActive()) {
                        autoState = State.HALT;
                    } else if(currentTime.time() - startTime  > robot.PARK_TIME) {
                        autoState = State.PARK;
                    } else {
                        autoState = State.SCORE_HIGH_JUNCTION1;
                    }
                    break;

                case SCORE_HIGH_JUNCTION1:
                    // drive to the 2nd low junction
                    if(opModeIsActive()) drive.followTrajectory(trajScoreHighJunction1);

                    // lower the lift to score the cone and release the cone
                    if(opModeIsActive()) mechs.liftPosition(robot.LIFT_CONE2, robot.LIFT_DOWN_POWER);
                    sleep(300);
                    if(opModeIsActive()) mechs.openClaw();
                    sleep(200);

                    if(!opModeIsActive()) {
                        autoState = State.HALT;
                    } else if(currentTime.time() - startTime  > robot.PARK_TIME) {
                        autoState = State.PARK;
                    } else {
                        autoState = State.RETRIEVE_CONE2;
                    }
                    break;

                case RETRIEVE_CONE2:
                    // drive to the cone stack
                    if(opModeIsActive()) drive.followTrajectory(trajRetrieveCone2);

                    // grab the cone and lift
                    if(opModeIsActive()) mechs.closeClaw();
                    sleep(300);
                    if(opModeIsActive()) mechs.liftPosition(robot.LIFT_JUNCTION_HIGH, robot.LIFT_UP_POWER);
                    sleep(200);

                    if(!opModeIsActive()) {
                        autoState = State.HALT;
                    } else if(currentTime.time() - startTime  > robot.PARK_TIME) {
                        autoState = State.PARK;
                    } else {
                        autoState = State.SCORE_HIGH_JUNCTION2;
                    }
                    break;

                case SCORE_HIGH_JUNCTION2:
                    // drive to the 2nd low junction
                    if(opModeIsActive()) drive.followTrajectory(trajScoreHighJunction2);

                    // lower the lift to score the cone and release the cone
                    if(opModeIsActive()) mechs.liftPosition(robot.LIFT_RESET, robot.LIFT_DOWN_POWER);
                    sleep(300);
                    if(opModeIsActive()) mechs.openClaw();
                    sleep(200);

                    if(!opModeIsActive()) {
                        autoState = State.HALT;
                    } else if(currentTime.time() - startTime  > robot.PARK_TIME) {
                        autoState = State.PARK;
                    } else {
                        autoState = State.RETRIEVE_CONE1;
                    }
                    break;

                case RETRIEVE_CONE1:
                    // drive to the cone stack
                    if(opModeIsActive()) drive.followTrajectory(trajRetrieveCone1);

                    // grab the cone and lift
                    if(opModeIsActive()) mechs.closeClaw();
                    sleep(300);
                    if(opModeIsActive()) mechs.liftPosition(robot.LIFT_JUNCTION_HIGH, robot.LIFT_UP_POWER);
                    sleep(200);

                    if(!opModeIsActive()) {
                        autoState = State.HALT;
                    } else if(currentTime.time() - startTime  > robot.PARK_TIME) {
                        autoState = State.PARK;
                    } else {
                        autoState = State.SCORE_MID1;
                    }
                    break;

                case SCORE_HIGH_JUNCTION3:
                    // drive to the 2nd low junction
                    if(opModeIsActive()) drive.followTrajectory(trajScoreHighJunction3);

                    // lower the lift to score the cone and release the cone
                    if(opModeIsActive()) mechs.liftPosition(robot.LIFT_RESET, robot.LIFT_DOWN_POWER);
                    sleep(300);
                    if(opModeIsActive()) mechs.openClaw();
                    sleep(200);

                    if(!opModeIsActive()) {
                        autoState = State.HALT;
                    } else if(currentTime.time() - startTime  > robot.PARK_TIME) {
                        autoState = State.PARK;
                    } else {
                        autoState = State.PARK;
                    }
                    break;

                case PARK:
                    if(opModeIsActive() && (currentTime.time() - startTime  < 29.5)) {
                        if (parkPosition == 1) {
                            // reset the lift
                            if(opModeIsActive()) mechs.liftPosition(robot.LIFT_RESET, robot.LIFT_DOWN_POWER);
                            if(opModeIsActive()) mechs.openClaw();

                            if(opModeIsActive()) drive.followTrajectory(trajPark1);

                        } else if (parkPosition == 2) {
                            if(opModeIsActive()) mechs.liftPosition(robot.LIFT_RESET, robot.LIFT_DOWN_POWER);
                            if(opModeIsActive()) mechs.openClaw();

                            if(opModeIsActive()) drive.followTrajectory(trajPark2);
                        } else {
                            if(opModeIsActive()) mechs.liftPosition(robot.LIFT_RESET, robot.LIFT_DOWN_POWER);
                            if(opModeIsActive()) mechs.openClaw();

                            if(opModeIsActive()) drive.followTrajectory(trajPark3);
                        }
                    }

                    while(opModeIsActive() && robot.motorLiftFront.getCurrentPosition() > 10){
                        if(opModeIsActive()) mechs.liftPosition(robot.LIFT_RESET, robot.LIFT_DOWN_POWER);
                    }

                    autoState = State.HALT;

                    break;

                case HALT:

                    robot.motorLiftFront.setPower(0);
                    robot.motorLiftRear.setPower(0);
                    if(opModeIsActive()) mechs.openClaw();

                    // End the program
                    requestOpModeStop();

                    break;
            }   // end of the switch state
        } // end of while(opModeIsActive())

        // End the program
        requestOpModeStop();
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
        TEST, DETECT_CONE, SCORE_LOW, RETRIEVE_CONE5, SCORE_LOW_JUNCTION2, RETRIEVE_CONE4,
        RETRIEVE_CONE3, RETRIEVE_CONE2, RETRIEVE_CONE1, SCORE_MID1, SCORE_HIGH_JUNCTION1,
        SCORE_HIGH_JUNCTION2, SCORE_HIGH_JUNCTION3, PARK, PARK2, HALT;
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