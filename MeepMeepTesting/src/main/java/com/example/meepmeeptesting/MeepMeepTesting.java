package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 7.513779527559055)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-29,-66,Math.toRadians(90)))
                                .lineToSplineHeading(new Pose2d(-27,-55,Math.toRadians(62.5)))
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {})
                                .UNSTABLE_addTemporalMarkerOffset(0.5, () ->{})
                                .waitSeconds(0.5)
                                .back(6)
                                .splineToSplineHeading(new Pose2d(-36, -47, Math.toRadians(90)), Math.toRadians(90))
                                .forward(36)
                                .back(12)
                                .splineToSplineHeading(new Pose2d(-60, -12, Math.toRadians(180)), Math.toRadians(180))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}