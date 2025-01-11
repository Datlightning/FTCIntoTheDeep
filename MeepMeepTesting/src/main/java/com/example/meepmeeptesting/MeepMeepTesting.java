package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.MinMax;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.ColorScheme;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import org.jetbrains.annotations.NotNull;

import java.awt.Color;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity sampleBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 15)
                .build();
        RoadRunnerBotEntity specimenBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 15)
                .setColorScheme(new ColorScheme() {
                    @NotNull
                    @Override
                    public Color getUI_MAIN_BG() {
                        return new Color(0,0,100);
                    }

                    @NotNull
                    @Override
                    public Color getTRAJECTORY_TEXT_COLOR() {
                        return new Color(0,0,0);
                    }

                    @NotNull
                    @Override
                    public Color getTRAJECTORY_SLIDER_FG() {
                        return new Color(0,0,255);
                    }

                    @NotNull
                    @Override
                    public Color getTRAJECTORY_SLIDER_BG() {
                        return new Color(255,255,255);
                    }

                    @NotNull
                    @Override
                    public Color getTRAJECTORY_MARKER_COLOR() {
                        return new Color(0,100,100);
                    }

                    @Override
                    public boolean isDark() {
                        return false;
                    }

                    @NotNull
                    @Override
                    public Color getBOT_BODY_COLOR() {
                        return new Color(0,155,200);
                    }

                    @NotNull
                    @Override
                    public Color getBOT_WHEEL_COLOR() {
                        return new Color(0,0,100);
                    }

                    @NotNull
                    @Override
                    public Color getBOT_DIRECTION_COLOR() {
                        return new Color(0,0,185);
                    }

                    @NotNull
                    @Override
                    public Color getAXIS_X_COLOR() {
                        return new Color(0,50,255);
                    }

                    @NotNull
                    @Override
                    public Color getAXIS_Y_COLOR() {
                        return new Color(0,50,255);
                    }

                    @Override
                    public double getAXIS_NORMAL_OPACITY() {
                        return 0.7;
                    }

                    @Override
                    public double getAXIS_HOVER_OPACITY() {
                        return 0.3;
                    }

                    @NotNull
                    @Override
                    public Color getTRAJECTORY_PATH_COLOR() {
                        return  new Color(0,80,255);
                    }

                    @NotNull
                    @Override
                    public Color getTRAJECTORY_TURN_COLOR() {
                        return new Color(0,80,255);
                    }

                })
                .build();
        RoadRunnerBotEntity sampleBot2 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 15)
                .setColorScheme(new ColorScheme() {
                    @NotNull
                    @Override
                    public Color getUI_MAIN_BG() {
                        return new Color(0,100,0);
                    }

                    @NotNull
                    @Override
                    public Color getTRAJECTORY_TEXT_COLOR() {
                        return new Color(0,0,0);
                    }

                    @NotNull
                    @Override
                    public Color getTRAJECTORY_SLIDER_FG() {
                        return new Color(0,255,0);
                    }

                    @NotNull
                    @Override
                    public Color getTRAJECTORY_SLIDER_BG() {
                        return new Color(255,255,255);
                    }

                    @NotNull
                    @Override
                    public Color getTRAJECTORY_MARKER_COLOR() {
                        return new Color(0,100,20);
                    }

                    @Override
                    public boolean isDark() {
                        return false;
                    }

                    @NotNull
                    @Override
                    public Color getBOT_BODY_COLOR() {
                        return new Color(0,155,0);
                    }

                    @NotNull
                    @Override
                    public Color getBOT_WHEEL_COLOR() {
                        return new Color(0,50,0);
                    }

                    @NotNull
                    @Override
                    public Color getBOT_DIRECTION_COLOR() {
                        return new Color(0, 100, 0);
                    }

                    @NotNull
                    @Override
                    public Color getAXIS_X_COLOR() {
                        return new Color(0,200,50);
                    }

                    @NotNull
                    @Override
                    public Color getAXIS_Y_COLOR() {
                        return new Color(0,200,50);
                    }

                    @Override
                    public double getAXIS_NORMAL_OPACITY() {
                        return 0.7;
                    }

                    @Override
                    public double getAXIS_HOVER_OPACITY() {
                        return 0.3;
                    }

                    @NotNull
                    @Override
                    public Color getTRAJECTORY_PATH_COLOR() {
                        return  new Color(0,255,0);
                    }

                    @NotNull
                    @Override
                    public Color getTRAJECTORY_TURN_COLOR() {
                        return new Color(0,255,0);
                    }

                })
                .build();
        AccelConstraint highMode = (robotPose, _path, _disp) -> {
            if (robotPose.position.y.value() < -20.0) {
                return new MinMax(-10,50);
            } else {
                return new MinMax(-30,50);
            }
        };
        Pose2d basket = new Pose2d(-55, -55, Math.toRadians(45));

        AccelConstraint smartScore = (robotPose, _path, _disp) -> {
            if (robotPose.position.y.value() < -32.0) {
                return new MinMax(-20,80);
            } else {
                return new MinMax(-30,80);
            }
        };
        AccelConstraint pickSampleAccel = (robotPose, _path, _disp) -> {
            if (robotPose.position.y.value() < -42.0) {
                return new MinMax(-10,22);
            } else {
                return new MinMax(-30,50);
            }
        };
        VelConstraint highModeVel = (robotPose, _path, _disp) -> {
            if (robotPose.position.y.value() < -25.0) {
                return 20;
            } else {
                return 50;
            }
        };



        sampleBot.runAction(sampleBot.getDrive().actionBuilder(new Pose2d(-32.5, -64 , Math.toRadians(0)))
                .setReversed(true)
//                .afterTime(0.5, new InstantAction(() -> intake.moveWrist(RobotConstants.floor_pickup_position)))
                .setTangent(Math.toRadians(135))
                .splineToConstantHeading(new Vector2d(-39, -60), Math.toRadians(135))
                .splineToSplineHeading(basket, Math.toRadians(135), new TranslationalVelConstraint(40), new ProfileAccelConstraint(-12,40))
                .splineToLinearHeading(new Pose2d(-57.5, -48, Math.toRadians(118.5)), Math.toRadians(90), new TranslationalVelConstraint(40),  new ProfileAccelConstraint(-12,22))
                        .setTangent(Math.toRadians(298.5))
                .splineToLinearHeading(basket, Math.toRadians(225), new TranslationalVelConstraint(36))
                .setReversed(false)
//                .stopAndAdd( new InstantAction(() -> intake.moveWrist(RobotConstants.floor_pickup_position)))
//                .afterTime(0.1, new InstantAction(pickupAfterDistance2::enable))
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(-48, -54, Math.toRadians(90)), Math.toRadians(0), new TranslationalVelConstraint(30), new ProfileAccelConstraint(-12,22))
//                .splineToSplineHeading(new Pose2d(-48, -35, Math.toRadians(90)), Math.toRadians(90), new TranslationalVelConstraint(20), new ProfileAccelConstraint(-8,10))

//                .stopAndAdd(new InstantAction(pickupAfterDistance2::failover))

                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-54.5, -59, Math.toRadians(45)), Math.toRadians(225), new TranslationalVelConstraint(30), new ProfileAccelConstraint(-12,30))

                .setReversed(false)
//                .stopAndAdd( new InstantAction(() -> intake.moveWrist(RobotConstants.floor_pickup_position)))
                .setTangent(Math.toRadians(135))
//                .afterTime(0.1, new InstantAction(pickupAfterDistance3::enable))
                .splineToLinearHeading(new Pose2d(-59, -54, Math.toRadians(90)), Math.toRadians(90), new TranslationalVelConstraint(40),  new ProfileAccelConstraint(-12,22))

//                .splineToSplineHeading(new Pose2d(-59, -35, Math.toRadians(90)), Math.toRadians(90), new TranslationalVelConstraint(10), new ProfileAccelConstraint(-10,12))
//                .stopAndAdd(new InstantAction(pickupAfterDistance3::failover))

                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-55, -59, Math.toRadians(45)), Math.toRadians(225), new TranslationalVelConstraint(35), new ProfileAccelConstraint(-12,30))

                .setReversed(false)
                .splineToSplineHeading(new Pose2d(-52.5, -30.5, Math.toRadians(180)), Math.toRadians(90), new TranslationalVelConstraint(40), pickSampleAccel)
//                .afterTime(0.1, new InstantAction(pickupAfterDistance4::enable))
                .splineToConstantHeading(new Vector2d(-61,-27), Math.toRadians(180), new TranslationalVelConstraint(10), new ProfileAccelConstraint(-10,15))
//                .stopAndAdd(new InstantAction(pickupAfterDistance4::failover))

                .setReversed(true)
                .setTangent(Math.toRadians(270))
                .splineToSplineHeading(new Pose2d(-50, -50, Math.toRadians(45)), Math.toRadians(270), new TranslationalVelConstraint(30), new ProfileAccelConstraint(-12,32))
                .splineToConstantHeading(new Vector2d(-57, -57), Math.toRadians(225), new TranslationalVelConstraint(10), new ProfileAccelConstraint(-12,32))

                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-50, -59), Math.toRadians(45))

                .build());
        sampleBot2.runAction(sampleBot.getDrive().actionBuilder(new Pose2d(-10, -66, Math.toRadians(270)))
                .setReversed(true)
//                .afterTime(1, new InstantAction(() -> intake.moveWrist(RobotConstants.specimen_deliver)))
                .splineToConstantHeading(new Vector2d(-6,-35.5), Math.toRadians(90), new TranslationalVelConstraint(40), new ProfileAccelConstraint(-10,30))
                                .waitSeconds(0.2)//delete this or smth alledgedly
//                .afterDisp(7.2, new InstantAction(() -> intake.openClaw()))
                .splineToConstantHeading(new Vector2d(-6, -34), Math.toRadians(270))
                .setReversed(false)
                .splineTo(new Vector2d(-6, -43), Math.toRadians(270))
//                .stopAndAdd( new InstantAction(() -> intake.moveWrist(RobotConstants.floor_pickup_position)))
//                .afterTime(0.1, intake.slideAction(0))
//                .afterTime(0.1, intake.armAction(0))
//                .stopAndAdd( new InstantAction(() -> intake.moveClaw(RobotConstants.claw_floor_pickup)))
                .splineTo(new Vector2d(-48, -44), Math.toRadians(90), new TranslationalVelConstraint(60), new ProfileAccelConstraint(-10,50))
//                .afterTime(0.1, new InstantAction(pickupAfterDistance1::enable))
                .splineTo(new Vector2d(-48, -40), Math.toRadians(90), new TranslationalVelConstraint(50), new ProfileAccelConstraint(-10,50))
//                .stopAndAdd(new InstantAction(pickupAfterDistance1::failover));

                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-54.5, -59, Math.toRadians(45)), Math.toRadians(225), new TranslationalVelConstraint(30), new ProfileAccelConstraint(-12,30))

                .setReversed(false)
//                .stopAndAdd( new InstantAction(() -> intake.moveWrist(RobotConstants.floor_pickup_position)))
                .setTangent(Math.toRadians(135))
//                .afterTime(0.1, new InstantAction(pickupAfterDistance3::enable))
                .splineToSplineHeading(new Pose2d(-59, -43, Math.toRadians(90)), Math.toRadians(90), new TranslationalVelConstraint(40),  new ProfileAccelConstraint(-12,22))
                .splineToSplineHeading(new Pose2d(-59, -35, Math.toRadians(90)), Math.toRadians(90), new TranslationalVelConstraint(10), new ProfileAccelConstraint(-10,12))
//                .stopAndAdd(new InstantAction(pickupAfterDistance3::failover))

                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-55, -59, Math.toRadians(45)), Math.toRadians(225), new TranslationalVelConstraint(35), new ProfileAccelConstraint(-12,30))

                .setReversed(false)
                .splineToSplineHeading(new Pose2d(-52.5, -30.5, Math.toRadians(180)), Math.toRadians(90), new TranslationalVelConstraint(40), pickSampleAccel)
//                .afterTime(0.1, new InstantAction(pickupAfterDistance4::enable))
                .splineToConstantHeading(new Vector2d(-61,-27), Math.toRadians(180), new TranslationalVelConstraint(10), new ProfileAccelConstraint(-10,15))
//                .stopAndAdd(new InstantAction(pickupAfterDistance4::failover))

                .setReversed(true)
                .setTangent(Math.toRadians(270))
                .splineToSplineHeading(new Pose2d(-50, -50, Math.toRadians(45)), Math.toRadians(270), new TranslationalVelConstraint(30), new ProfileAccelConstraint(-12,32))
                .splineToConstantHeading(new Vector2d(-57, -57), Math.toRadians(225), new TranslationalVelConstraint(10), new ProfileAccelConstraint(-12,32))

                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-50, -59), Math.toRadians(45))

                .build());
        specimenBot.runAction(specimenBot.getDrive().actionBuilder(new Pose2d(10, -64 , Math.toRadians(270)))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(0,-30.5), Math.toRadians(90), null, smartScore)

                .setReversed(true)
                .setTangent(Math.toRadians(270))
                .splineTo(new Vector2d(0, -42), Math.toRadians(270))

                .splineToSplineHeading(new Pose2d(10, -43,0), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(35, -31), Math.toRadians(90), highModeVel, highMode)
                .splineToConstantHeading(new Vector2d(40, -17), Math.toRadians(0), highModeVel, highMode)
                .splineToConstantHeading(new Vector2d(44, -18), Math.toRadians(270), highModeVel, highMode)

                .lineToYConstantHeading(-52, new TranslationalVelConstraint(120))
                .setTangent(Math.toRadians(90))
                .lineToYConstantHeading(-28, new TranslationalVelConstraint(50))
                .splineToConstantHeading(new Vector2d(57, -17), Math.toRadians(270), highModeVel, highMode)
                .lineToYConstantHeading(-52, new TranslationalVelConstraint(120))
                .setTangent(Math.toRadians(90))

//                .splineToLinearHeading(new Pose2d(32, -45, Math.toRadians(90)), Math.toRadians(270), new TranslationalVelConstraint(120))

                .splineToSplineHeading(new Pose2d(52, -44, Math.toRadians(0)), Math.toRadians(180), new TranslationalVelConstraint(120), new ProfileAccelConstraint(-14,40))
                .splineToLinearHeading(new Pose2d(32, -54, Math.toRadians(0)), Math.toRadians(270), new TranslationalVelConstraint(120), new ProfileAccelConstraint(-14,40))

                                .waitSeconds(100)
                                .setTangent(Math.toRadians(0))
                .lineToX(42, new TranslationalVelConstraint(120), new ProfileAccelConstraint(-20,80))

                .setReversed(true)
//                .afterTime(0.5, new InstantAction(() -> intake.moveWrist(RobotConstants.specimen_deliver)))
                .splineToLinearHeading(new Pose2d(2, -30.5, Math.toRadians(270)), Math.toRadians(90),  new TranslationalVelConstraint(120),smartScore)

                .setReversed(false)
                .splineToSplineHeading(new Pose2d(2, -42, Math.toRadians(270)), Math.toRadians(270))
//                .afterTime(0.1, new InstantAction(pickupAfterDistance2::enable))
                .splineToSplineHeading(new Pose2d(25, -64, Math.toRadians(0)), Math.toRadians(0), new TranslationalVelConstraint(120), new ProfileAccelConstraint(-30,80))

                .lineToX(42, new TranslationalVelConstraint(120), new ProfileAccelConstraint(-40,80))
//                .stopAndAdd(new InstantAction(pickupAfterDistance2::failover))

                .setReversed(true)
//                .afterTime(0.5, new InstantAction(() -> intake.moveWrist(RobotConstants.specimen_deliver)))
                .splineToLinearHeading(new Pose2d(7, -30.5, Math.toRadians(270)), Math.toRadians(90),  new TranslationalVelConstraint(120), smartScore)

                .setReversed(false)
                .splineToSplineHeading(new Pose2d(7, -42, Math.toRadians(270)), Math.toRadians(270))
//                .afterTime(0.1, new InstantAction(pickupAfterDistance3::enable))
                .splineToSplineHeading(new Pose2d(25, -64, Math.toRadians(0)), Math.toRadians(0), new TranslationalVelConstraint(120), new ProfileAccelConstraint(-30,80))

                .lineToX(42, new TranslationalVelConstraint(120), new ProfileAccelConstraint(-40,80))
//                .stopAndAdd(new InstantAction(pickupAfterDistance3::failover))

                .setReversed(true)
//                .afterTime(0.5, new InstantAction(() -> intake.moveWrist(RobotConstants.specimen_deliver)))
                .splineToLinearHeading(new Pose2d(10, -30.5, Math.toRadians(270)), Math.toRadians(90),  new TranslationalVelConstraint(120), smartScore)

                .splineToConstantHeading(new Vector2d(10,-40), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(64,-64), Math.toRadians(0))

                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL )
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(sampleBot)
                .addEntity(sampleBot2)
                .addEntity(specimenBot)
                .start();
    }
}