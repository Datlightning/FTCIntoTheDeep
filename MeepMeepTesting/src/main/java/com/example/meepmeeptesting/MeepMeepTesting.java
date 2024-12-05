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

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-7.125, -64 , Math.toRadians(180)))
                .setReversed(true)
                .strafeTo(new Vector2d(-7.125, -58))
                .splineToSplineHeading(new Pose2d(0, -32, Math.toRadians(270)), Math.toRadians(90), new TranslationalVelConstraint(40), new ProfileAccelConstraint(-10,30))
                .setReversed(false)
                .splineTo(new Vector2d(0, -43), Math.toRadians(270))
                .splineTo(new Vector2d(-48, -39), Math.toRadians(90), new TranslationalVelConstraint(40), new ProfileAccelConstraint(-18,40))

//                .splineToSplineHeading(new Pose2d(-5.125, -32, Math.toRadians(270)), Math.toRadians(90), new TranslationalVelConstraint(40), new ProfileAccelConstraint(-8,22))
//                .splineToLinearHeading(new Pose2d(-53, -57, Math.toRadians(45)), Math.toRadians(225))
//                .splineToSplineHeading(new Pose2d(-50,-32, Math.toRadians(180)), Math.toRadians(90), new TranslationalVelConstraint(30), new ProfileAccelConstraint(-12,22))
//                .splineToConstantHeading(new Vector2d(-56,-28), Math.toRadians(180), new TranslationalVelConstraint(10), new ProfileAccelConstraint(-10,15))
//                .setReversed(false)
////                .splineTo(new Vector2d(-7.125, -40), Math.toRadians(180))
//                .splineToSplineHeading(new Pose2d(-47.5, -39, Math.toRadians(90)), Math.toRadians(90), new TranslationalVelConstraint(50), new ProfileAccelConstraint(-10,30))
//                .splineToConstantHeading(new Vector2d(36, -46), Math.toRadians(0))
//                .splineToConstantHeading(new Vector2d(48, -34.5), Math.toRadians(90))
//                .splineToConstantHeading(new Vector2d(52, -40), Math.toRadians(0))
//                .splineToConstantHeading(new Vector2d(58, -34.5), Math.toRadians(90))
//                .waitSeconds(1)
//                .setReversed(false)
//                .splineToSplineHeading(new Pose2d(55, -14, 0), Math.toRadians(0))
//                .splineToConstantHeading(new Vector2d(55 , -14), Math.toRadians(0))
//                .splineToConstantHeading(new Vector2d(60, -50), Math.toRadians(270))
//                .splineToSplineHeading(new Pose2d(28, -58, Math.toRadians(0)), Math.toRadians(180))
//                .setReversed(true)
//                .splineToLinearHeading(new Pose2d(10, -34.5, Math.toRadians(270)), Math.toRadians(90))
//                .setReversed(false)
//                .splineToLinearHeading(new Pose2d(28, -58, Math.toRadians(0)), Math.toRadians(0))
//                .setReversed(true)
//                .splineToLinearHeading(new Pose2d(7, -34.5, Math.toRadians(270)), Math.toRadians(90))
//                .setReversed(false)
//                .splineToLinearHeading(new Pose2d(28, -58, Math.toRadians(0)), Math.toRadians(0))
//                .setReversed(true)
//                .splineToLinearHeading(new Pose2d(4, -34.5, Math.toRadians(270)), Math.toRadians(90))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL )
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}