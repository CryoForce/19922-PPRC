package com.ironlions.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity bronto = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 7)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(36, -60, Math.toRadians(90)))
                                .strafeLeft(36)
                                .forward(13)
                                .forward(11)

//                                .strafeLeft(10)
//                                .strafeRight(10)
//                                .strafeLeft(10)
//                                .strafeRight(10)
//                                .strafeLeft(10)
//                                .strafeRight(10)
//                                .strafeLeft(10)
//                                .strafeRight(10)
//                                .strafeLeft(10)
//                                .strafeRight(10)
//                                .forward(10)
//                                .turn(Math.toRadians(90))
                                .build()
                );

        bronto.setLooping(false);
        
        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(bronto)
                .start();
    }
}
