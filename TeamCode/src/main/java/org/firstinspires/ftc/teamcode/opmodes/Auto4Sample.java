package org.firstinspires.ftc.teamcode.opmodes;

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
public class Auto4Sample extends NGAutoOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(-32.5, -64, Math.toRadians(0));
        double INCHES_FORWARD = -2.5;
        initAuto(beginPose);
        AccelConstraint pickSampleAccel = (robotPose, _path, _disp) -> {
            if (robotPose.position.y.value() > -30.0) {
                return new MinMax(-10,12);
            } else {
                return new MinMax(-30,50);
            }
        };


        Pose2d basket = new Pose2d(-57.5, -56.5 + INCHES_FORWARD, Math.toRadians(45));
        Pose2d sample2Pickup = new Pose2d(-48, -50.5 + INCHES_FORWARD, Math.toRadians(90));
        Pose2d sample3Pickup = new Pose2d(-59, -50.5 + INCHES_FORWARD, Math.toRadians(90));
        Pose2d sample4Pickup = new Pose2d(-58 , -48.5 + INCHES_FORWARD, Math.toRadians(118.5));

        TrajectoryActionBuilder firstSamplePath = drive.closeActionBuilder(beginPose)
                .setReversed(true)
                .afterTime(0.5, new InstantAction(() -> intake.moveWrist(RobotConstants.floor_pickup_position)))
                .setTangent(Math.toRadians(135))
                .splineToConstantHeading(new Vector2d(-39, -60), Math.toRadians(135))
                .splineToSplineHeading(basket, Math.toRadians(135), new TranslationalVelConstraint(24), new ProfileAccelConstraint(-10,40));

        TrajectoryActionBuilder secondSamplePath = firstSamplePath.endTrajectory().fresh()
                .setReversed(false)
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(sample2Pickup, Math.toRadians(0), new TranslationalVelConstraint(34), new ProfileAccelConstraint(-12,32));

        TrajectoryActionBuilder scoreSecondSamplePath = drive.closeActionBuilder(sample2Pickup)
                .setReversed(true)
                .setTangent(Math.PI + Math.atan2(sample2Pickup.position.y - basket.position.y, sample2Pickup.position.x - basket.position.x))
                .splineToLinearHeading(basket, Math.toRadians(225), new TranslationalVelConstraint(36), new ProfileAccelConstraint(-10,40));

        TrajectoryActionBuilder thirdSamplePath = scoreSecondSamplePath.endTrajectory().fresh()
                .setReversed(false)
                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(sample3Pickup, Math.toRadians(90), new TranslationalVelConstraint(34),  new ProfileAccelConstraint(-12,32));

        TrajectoryActionBuilder scoreThirdSamplePath = drive.closeActionBuilder(sample3Pickup)
                .setReversed(true)
                .setTangent(Math.PI + Math.atan2(sample3Pickup.position.y - basket.position.y, sample3Pickup.position.x - basket.position.x))
                .splineToLinearHeading(basket, Math.toRadians(225), new TranslationalVelConstraint(36), new ProfileAccelConstraint(-10,40));

        TrajectoryActionBuilder fourthSamplePath = scoreThirdSamplePath.endTrajectory().fresh()
                .setReversed(false)
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(sample4Pickup, Math.toRadians(90), new TranslationalVelConstraint(34),  new ProfileAccelConstraint(-12,32));
        TrajectoryActionBuilder scoreFourthSamplePath = drive.closeActionBuilder(sample4Pickup)
                .setReversed(true)
                .setTangent(Math.toRadians(298.5))
                .splineToLinearHeading(basket, Math.toRadians(225), new TranslationalVelConstraint(36), new ProfileAccelConstraint(-10,40));

        TrajectoryActionBuilder parkPath = scoreFourthSamplePath.endTrajectory().fresh()
                .setReversed(true)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-34, -18, Math.toRadians(0)), Math.toRadians(0), new TranslationalVelConstraint(40), new ProfileAccelConstraint(-10,40));

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
                                goToSampleWithSlides(fourthSample),
                                slideCollectSampleAndScore(scoreFourthSample, RobotConstants.claw_open),
                                new ParallelAction(
                                        new SequentialAction(
                                                new SleepAction(1),
                                                new InstantAction(() -> intake.moveWrist(0))
                                        ),
                                        new SequentialAction(
                                            intake.slideAction(0),
                                            intake.armAction(0)
                                        ),
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