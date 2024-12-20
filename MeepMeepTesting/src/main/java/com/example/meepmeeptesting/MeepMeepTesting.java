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
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-33, -64 , Math.toRadians(0)))
                        .setReversed(true)
//                .splineToLinearHeading(new Pose2d(-52, -54, Math.toRadians(45)), Math.toRadians(135), new TranslationalVelConstraint(40), new ProfileAccelConstraint(-22,40))
//                        .waitSeconds(1)
                        .setTangent(Math.toRadians(135))
                .splineToConstantHeading(new Vector2d(-39, -60), Math.toRadians(135))
                .splineToSplineHeading(new Pose2d(-52, -54, Math.toRadians(45)), Math.toRadians(135), new TranslationalVelConstraint(40), new ProfileAccelConstraint(-12,40))
                //                .splineToSplineHeading(new Pose2d(-50,-33, Math.toRadians(180)), Math.toRadians(90), new TranslationalVelConstraint(35), new ProfileAccelConstraint(-12,30))
//                .splineToConstantHeading(new Vector2d(-56,-27), Math.toRadians(180), new TranslationalVelConstraint(15), new ProfileAccelConstraint(-8,20))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL )
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}