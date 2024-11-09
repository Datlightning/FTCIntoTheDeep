package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(44, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-38, -64 , Math.toRadians(0  )))
                        .setReversed(true)
                .splineToConstantHeading(new Vector2d(-43, -60), Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(-48, -54, Math.toRadians(45)), Math.toRadians(135), new TranslationalVelConstraint(10))



//                .setReversed(true)
//                .splineToSplineHeading(new Pose2d(-40,-32, Math.toRadians(180)), Math.toRadians(90))
//                .splineToConstantHeading(new Vector2d(-43,-28.5), Math.toRadians(180))
//                .splineTo(new Vector2d(-50,-28.5), Math.toRadians(180))
                .build());
//        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(0,0,0))
//                .splineTo(new Vector2d(30, 30), 0)
//                        .setReversed(true)
//                .splineTo(new Vector2d(0, 0), Math.PI)
//                .build());
        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL )
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}