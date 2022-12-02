package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "Auto: Red Corner", group = "Concept")
@Disabled

public class AutoBlueTerminalSplines extends LinearOpMode {
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose=new Pose2d(-29,-66,Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        TrajectorySequence myTrajectory = drive.trajectorySequenceBuilder(new Pose2d(-29,-66,Math.toRadians(90)))
                .splineTo(new Vector2d(-30,-55),Math.toRadians(55))
                .splineTo(new Vector2d(-29,-66),Math.toRadians(90))
                .forward(70)
                .back(24)
                .splineTo(new Vector2d(-60,-20),Math.toRadians(180))
                .build();

        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectorySequence(myTrajectory);
    }
}

