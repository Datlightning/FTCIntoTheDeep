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
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
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


        Pose2d basket = new Pose2d(-57.7, -57.7, Math.toRadians(45));
        TrajectoryActionBuilder firstSamplePath = drive.closeActionBuilder(beginPose)
                .setReversed(true)
                .afterTime(0.5, new InstantAction(() -> intake.moveWrist(RobotConstants.floor_pickup_position)))
                .setTangent(Math.toRadians(135))
                .splineToConstantHeading(new Vector2d(-39, -60), Math.toRadians(135))
                .splineToSplineHeading(basket, Math.toRadians(135), new TranslationalVelConstraint(30));

        TrajectoryActionBuilder secondSamplePath = firstSamplePath.endTrajectory().fresh()
                .setReversed(false)
                .stopAndAdd( new InstantAction(() -> intake.moveWrist(RobotConstants.floor_pickup_position)))
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(-48, -54, Math.toRadians(90)), Math.toRadians(0), new TranslationalVelConstraint(30), new ProfileAccelConstraint(-12,22));

        TrajectoryActionBuilder scoreSecondSamplePath = drive.closeActionBuilder(new Pose2d(-48, -40, Math.toRadians(90)))
                .setReversed(true)
                .splineToLinearHeading(basket, Math.toRadians(225), new TranslationalVelConstraint(36));

        TrajectoryActionBuilder thirdSamplePath = scoreSecondSamplePath.endTrajectory().fresh()
                .setReversed(false)
                .stopAndAdd( new InstantAction(() -> intake.moveWrist(RobotConstants.floor_pickup_position)))
                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(new Pose2d(-59, -54, Math.toRadians(90)), Math.toRadians(90), new TranslationalVelConstraint(40),  new ProfileAccelConstraint(-12,22));


        TrajectoryActionBuilder scoreThirdSamplePath = drive.closeActionBuilder(new Pose2d(-59, -39, Math.toRadians(90)))
                .setReversed(true)
                .splineToLinearHeading(basket, Math.toRadians(225), new TranslationalVelConstraint(36));

        TrajectoryActionBuilder fourthSamplePath = scoreThirdSamplePath.endTrajectory().fresh()
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(-50.5, -33.5, Math.toRadians(180)), Math.toRadians(90), null, pickSampleAccel)
                .splineToConstantHeading(new Vector2d(-67,-25.5), Math.toRadians(180), new TranslationalVelConstraint(10), new ProfileAccelConstraint(-5,12));
        TrajectoryActionBuilder scoreFourthSamplePath = drive.actionBuilder(new Pose2d(-58, -28, Math.toRadians(180)))
                .setReversed(true)
                .setTangent(Math.toRadians(225))
                .splineToSplineHeading(new Pose2d(-50, -50, Math.toRadians(45)), Math.toRadians(270), new TranslationalVelConstraint(30), new ProfileAccelConstraint(-12,32))
                .splineToConstantHeading(basket.position, Math.toRadians(225), new TranslationalVelConstraint(10), new ProfileAccelConstraint(-12,32));
        TrajectoryActionBuilder parkPath = scoreFourthSamplePath.endTrajectory().fresh()
                .setReversed(true)
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-35, -12), Math.toRadians(0), new TranslationalVelConstraint(36));

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
                                intake.score(),
                                new InstantAction(() -> intake.moveClaw(RobotConstants.claw_floor_pickup)),
                                goToSampleWithSlides(secondSample),
                                collectSampleAndScore(scoreSecondSample, RobotConstants.claw_floor_pickup, false),
                                goToSampleWithSlides(thirdSample),
                                collectSampleAndScore(scoreThirdSample, RobotConstants.inside_pickup_open, false),
                                intake.armAction(400,800),
                                intake.slideAction(100),
                                new ParallelAction(
                                        fourthSample, intake.armAction(400),
                                        new InstantAction(() -> intake.distance.setOn(true)),
                                        new InstantAction(() -> intake.turnAndRotateClaw(180,0))
                                ),
                                new InstantAction(() -> intake.arm.setExitWithTime(true)),
                                intake.armAction(200),
                                new InstantAction(() -> intake.arm.setExitWithTime(false)),
                                intake.grab(RobotConstants.inside_pickup_closed),
                                intake.armAction(400),
                                new ParallelAction(
                                        intake.raiseArmButNotSnagOnBasket(),
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