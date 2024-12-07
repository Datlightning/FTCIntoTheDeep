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

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(10, -64 , Math.toRadians(180)))
                .setReversed(false)
                .strafeTo(new Vector2d(10, -58))
                .splineToSplineHeading(new Pose2d(6, -32, Math.toRadians(270)), Math.toRadians(90), new TranslationalVelConstraint(40), new ProfileAccelConstraint(-10,30))
                .setReversed(true)
                .waitSeconds(1)
                .splineToSplineHeading(new Pose2d(6, -40, Math.toRadians(270)), Math.toRadians(270))
                .splineToSplineHeading(new Pose2d(36, -20, Math.toRadians(0)), Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(48, -14, Math.toRadians(0)), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(52, -50), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(48, -14), 0)
                .splineToConstantHeading(new Vector2d(52, -16), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(52, -46), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(60, -13), 0)
                .splineToConstantHeading(new Vector2d(62, -15), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(61, -46), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(25, -58), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(10, -32, Math.toRadians(270)), Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(10, -42, Math.toRadians(270)), Math.toRadians(270))
                .splineToSplineHeading(new Pose2d(28, -58, Math.toRadians(0)), Math.toRadians(0))



                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL )
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}