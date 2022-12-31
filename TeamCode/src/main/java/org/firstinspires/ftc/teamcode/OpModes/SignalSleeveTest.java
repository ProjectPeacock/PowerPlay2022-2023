package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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

@TeleOp(name = "Custom Signal Sleeve Test", group = "Development")

public class SignalSleeveTest extends LinearOpMode {

    // set up the dashboard
    FtcDashboard dashboard;

    // set up Vuforia amd Tensorflow for signal sleeve sensing
    private static final String VUFORIA_KEY =
            "AfHl2GP/////AAABmeJc93xOhk1MvZeKbP5E43taYJ6kodzkhsk5wOLGwZI3wxf7v1iTx2Mem/VZSEtpxb3U2fMO7n0EUxSeHRWhOXeX16dMFcjfalezjo3ZkzBuG/y2r4kgLwKs4APyAIClBAon+tf/W/4NkTkYuHGo8zZ0slH/iBpqxvblpNURsG5h4VxPFgF5D/FIfmjnddzQpa4cGarle/Zvuah6q2orUswun31P6ZLuIJvdOIQf7o/ruoRygsSXfVYc35w+Xwm+bwjpZUNzHHYvRNrp0HNWC3Fr2hd0TqWKIIYlCoHj0m5OKX22Ris23V8PdKM/i4/ZIy8JewJXetv1rERC5bfHmUXCS4Rl7RjR+ZscQ5aA0nr8";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    private static final String TFOD_MODEL_ASSET = "model_20221208_140441.tflite";
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

        /** Wait for the game to begin which determining signal sleeve parking zone */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()) {
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

        // End the program
        requestOpModeStop();
    }   // end of runOpMode

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