package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.MinMax;
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
        AccelConstraint highMode = (robotPose, _path, _disp) -> {
            if (robotPose.position.y.value() < -20.0) {
                return new MinMax(-10,50);
            } else {
                return new MinMax(-30,50);
            }
        };
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(10, -64 , Math.toRadians(270)))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(0,-42), Math.toRadians(90))
                .setReversed(true)
                .waitSeconds(1)
                .splineToConstantHeading(new Vector2d(10, -42), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(30, -42), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(36, -20, Math.toRadians(0)), Math.toRadians(90), new TranslationalVelConstraint(20), new ProfileAccelConstraint(-14,20))
                .splineToSplineHeading(new Pose2d(44, -10, Math.toRadians(0)), Math.toRadians(270), new TranslationalVelConstraint(50), new ProfileAccelConstraint(-14,40))
                .splineToConstantHeading(new Vector2d(48, -52), Math.toRadians(270), null, highMode)
                .splineToConstantHeading(new Vector2d(46, -12), 0, null, highMode)
                .splineToConstantHeading(new Vector2d(55, -10), Math.toRadians(270), new TranslationalVelConstraint(50), new ProfileAccelConstraint(-14,40))
                .splineToConstantHeading(new Vector2d(55, -54), Math.toRadians(270), null, highMode)
                .splineToConstantHeading(new Vector2d(25, -58), Math.toRadians(180), new TranslationalVelConstraint(50), new ProfileAccelConstraint(-15,40))
//                .splineToLinearHeading(new Pose2d(10, -32, Math.toRadians(270)), Math.toRadians(90))
//                .splineToSplineHeading(new Pose2d(10, -42, Math.toRadians(270)), Math.toRadians(270))
//                .splineToSplineHeading(new Pose2d(28, -58, Math.toRadians(0)), Math.toRadians(0))



                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL )
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}