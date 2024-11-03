package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
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

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-12, -63, Math.toRadians(270 )))
                .setReversed(true)

                .splineToLinearHeading(new Pose2d(-10, -34, Math.toRadians(270)), Math.toRadians(90))
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(-48,-36, Math.toRadians(90)), Math.toRadians(90))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-53,-53, Math.toRadians(45)), Math.toRadians(225))
                .setReversed(false)

                .splineToLinearHeading(new Pose2d(-58,-36, Math.toRadians(90)), Math.toRadians(90))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-53,-53, Math.toRadians(45)), Math.toRadians(225))
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(-50,-40, Math.toRadians(180)), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-50,-24), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-56,-24), Math.toRadians(180))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-53,-53, Math.toRadians(45)), Math.toRadians(225))
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