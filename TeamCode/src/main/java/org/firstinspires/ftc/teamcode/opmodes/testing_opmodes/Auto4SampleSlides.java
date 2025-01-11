package org.firstinspires.ftc.teamcode.opmodes.testing_opmodes;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.MinMax;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TimeTrajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilderParams;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.opmodes.NGAutoOpMode;

@Autonomous
public class Auto4SampleSlides extends NGAutoOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(-32.5, -64, Math.toRadians(0));
        initAuto(beginPose);
        AccelConstraint pickSampleAccel = (robotPose, _path, _disp) -> {
            if (robotPose.position.y.value() > -30.0) {
                return new MinMax(-10,12);
            } else {
                return new MinMax(-30,50);
            }
        };


        Pose2d basket = new Pose2d(-58, -58, Math.toRadians(45));
        TrajectoryActionBuilder firstSamplePath = drive.closeActionBuilder(beginPose)
                .setReversed(true)
                .afterTime(0.5, new InstantAction(() -> intake.moveWrist(RobotConstants.floor_pickup_position)))
                .setTangent(Math.toRadians(135))
                .splineToConstantHeading(new Vector2d(-39, -60), Math.toRadians(135))
                .splineToSplineHeading(basket, Math.toRadians(135), new TranslationalVelConstraint(24), new ProfileAccelConstraint(-10,40));

        TrajectoryActionBuilder secondSamplePath = firstSamplePath.endTrajectory().fresh()
                .setReversed(false)
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(-48, -51.5, Math.toRadians(90)), Math.toRadians(0), new TranslationalVelConstraint(30), new ProfileAccelConstraint(-12,22));

        TrajectoryActionBuilder scoreSecondSamplePath = drive.closeActionBuilder(new Pose2d(-48, -51.5, Math.toRadians(90)))
                .setReversed(true)
                .splineToLinearHeading(basket, Math.toRadians(225), new TranslationalVelConstraint(36), new ProfileAccelConstraint(-10,40));

        TrajectoryActionBuilder thirdSamplePath = scoreSecondSamplePath.endTrajectory().fresh()
                .setReversed(false)
                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(new Pose2d(-59, -51.5, Math.toRadians(90)), Math.toRadians(90), new TranslationalVelConstraint(40),  new ProfileAccelConstraint(-12,22));

        TrajectoryActionBuilder scoreThirdSamplePath = drive.closeActionBuilder(new Pose2d(-59, -51.5, Math.toRadians(90)))
                .setReversed(true)
                .splineToLinearHeading(basket, Math.toRadians(225), new TranslationalVelConstraint(36), new ProfileAccelConstraint(-10,40));

        TrajectoryActionBuilder fourthSamplePath = scoreThirdSamplePath.endTrajectory().fresh()
                .setReversed(false)
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(-57.5, -48, Math.toRadians(118.5)), Math.toRadians(90), new TranslationalVelConstraint(40),  new ProfileAccelConstraint(-12,22));
        TrajectoryActionBuilder scoreFourthSamplePath = drive.actionBuilder(new Pose2d(-57.5, -48, Math.toRadians(118.5)))
                .setReversed(true)
                .setTangent(Math.toRadians(298.5))
                .splineToLinearHeading(basket, Math.toRadians(225), new TranslationalVelConstraint(36), new ProfileAccelConstraint(-10,40));

        TrajectoryActionBuilder parkPath = scoreFourthSamplePath.endTrajectory().fresh()
                .setReversed(true)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-30, -18, Math.toRadians(0)), Math.toRadians(0), new TranslationalVelConstraint(36));

        FailoverAction firstSample = new FailoverAction(firstSamplePath.build(), new InstantAction(() -> drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0))));
        FailoverAction secondSample = new FailoverAction( secondSamplePath.build(), new InstantAction(() -> drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0))));
        Action scoreSecondSample = scoreSecondSamplePath.build();
        FailoverAction thirdSample = new FailoverAction(thirdSamplePath.build(), new InstantAction(() -> drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0))));
        Action scoreThirdSample = scoreThirdSamplePath.build();
        FailoverAction fourthSample = new FailoverAction( fourthSamplePath.build(), new InstantAction(() -> drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0))));
        Action scoreFourthSample = scoreFourthSamplePath.build();
        Action park = parkPath.build();


        waitForStart();


        Actions.runBlocking(
                new ParallelAction(
                        intake.distance.updateAction(),
                        intake.updateAction(),
                        trafficLight.updateAction(),
                        new SequentialAction(
                                new ParallelAction(
                                        firstSample,
                                        new SequentialAction(
                                                new SleepAction(0.3),
                                                intake.raiseArm(false)
                                        )
                                ),
                                intake.scoreSlidePickup(),
                                new InstantAction(() -> intake.moveClaw(RobotConstants.claw_flat)),
                                goToSampleWithSlides(secondSample),
                                slideCollectSampleAndScore(scoreSecondSample, RobotConstants.claw_flat),
                                goToSampleWithSlides(thirdSample),
                                slideCollectSampleAndScore(scoreThirdSample, RobotConstants.claw_flat),
                                new ParallelAction(
                                        new SequentialAction(
                                            intake.armAction(240,800),
                                            new InstantAction(
                                                    () -> {
                                                        intake.moveWrist(180);
                                                        intake.moveClaw(RobotConstants.claw_flat);
                                                    }
                                            ),
                                            intake.slideAction(950)
                                        ),
                                    fourthSample
                                ),
                                intake.grab(),
                                new ParallelAction(
                                        intake.slideAction(400),
                                        new SequentialAction(
                                                intake.armAction(360),
                                                new InstantAction(() -> intake.moveWrist(90)),
                                                intake.raiseArm()
                                        ),
                                        scoreFourthSample
                                ),
                                intake.scoreAndFold(),
                                new ParallelAction(
                                        intake.armAction(0),
                                        park

                                )


                        )
                )
        );
        //Arm to 100
        //Slides to 650
        //Wrist to 0.45


    }
}