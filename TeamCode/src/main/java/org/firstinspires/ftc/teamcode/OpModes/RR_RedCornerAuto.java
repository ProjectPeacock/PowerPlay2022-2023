package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.HWProfile;
import org.firstinspires.ftc.teamcode.Libs.AutoClass;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "RR Auto: Red Corner", group = "Concept")
@Disabled

public class RR_RedCornerAuto extends LinearOpMode {

    public void runOpMode(){
        LinearOpMode opMode = this;
        HWProfile hwprofile = new HWProfile();
        AutoClass robot = new AutoClass(hwprofile,opMode);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-29,-66,Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        TrajectorySequence scoreCone1 = drive.trajectorySequenceBuilder(startPose)
                .lineToSplineHeading(new Pose2d(-27,-55,Math.toRadians(62.5)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {})
                .UNSTABLE_addTemporalMarkerOffset(0.5, () ->{})
                .waitSeconds(0.5)
                .back(6)
                .splineToSplineHeading(new Pose2d(-36, -47, Math.toRadians(90)), Math.toRadians(90))
                .build();

        TrajectorySequence returnHome = drive.trajectorySequenceBuilder(scoreCone1.end())
                .splineTo(new Vector2d(-29,-66),Math.toRadians(90))
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> robot.liftReset())
                .build();


        waitForStart();

        while(opModeIsActive()){
            drive.followTrajectorySequence(scoreCone1);
            drive.followTrajectorySequence(returnHome);
        }
    }
}
