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
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.RobotConstants;

@Autonomous
public class Auto4Sample extends NGAutoOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(-32.5, -64, Math.toRadians(0));
        initAuto(beginPose);
        AccelConstraint pickSampleAccel = (robotPose, _path, _disp) -> {
            if (robotPose.position.y.value() < -42.0) {
                return new MinMax(-10,22);
            } else {
                return new MinMax(-30,50);
            }
        };
        FailoverAction pickupAfterDistance2 = new FailoverAction(pickupAfterDistance(RobotConstants.TOO_FAR, intake.distance), new InstantAction(() -> intake.distance.setOn(false)), false);

        FailoverAction pickupAfterDistance3 = new FailoverAction(pickupAfterDistance(RobotConstants.TOO_FAR, intake.distance), new InstantAction(() -> intake.distance.setOn(false)), false);

        FailoverAction pickupAfterDistance4 = new FailoverAction(intake.distance.waitAction(4),new InstantAction(() -> intake.distance.setOn(false)), false );


        Pose2d basket = new Pose2d(-55, -55, Math.toRadians(45));
        TrajectoryActionBuilder firstSamplePath = drive.actionBuilder(beginPose)
                .setReversed(true)
                .afterTime(0.5, new InstantAction(() -> intake.moveWrist(RobotConstants.floor_pickup_position)))
                .setTangent(Math.toRadians(135))
                .splineToConstantHeading(new Vector2d(-39, -60), Math.toRadians(135))
                .splineToSplineHeading(basket, Math.toRadians(135), new TranslationalVelConstraint(40), new ProfileAccelConstraint(-12,40));

        TrajectoryActionBuilder secondSamplePath = firstSamplePath.endTrajectory().fresh()
                .setReversed(false)
                .stopAndAdd( new InstantAction(() -> intake.moveWrist(RobotConstants.floor_pickup_position)))
                .afterTime(0.1, new InstantAction(pickupAfterDistance2::enable))
                .splineToSplineHeading(new Pose2d(-48, -43, Math.toRadians(90)), Math.toRadians(90), new TranslationalVelConstraint(40), new ProfileAccelConstraint(-18,40))
                .splineToSplineHeading(new Pose2d(-48, -39, Math.toRadians(90)), Math.toRadians(90), new TranslationalVelConstraint(40), new ProfileAccelConstraint(-18,40))
                .stopAndAdd(new InstantAction(pickupAfterDistance2::failover));

        TrajectoryActionBuilder scoreSecondSamplePath = drive.closeActionBuilder(new Pose2d(-48, -39, Math.toRadians(90)))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-55, -57, Math.toRadians(45)), Math.toRadians(225), new TranslationalVelConstraint(50), new ProfileAccelConstraint(-12,30));

        TrajectoryActionBuilder thirdSamplePath = scoreSecondSamplePath.endTrajectory().fresh()
                .setReversed(false)
                .stopAndAdd( new InstantAction(() -> intake.moveWrist(RobotConstants.floor_pickup_position)))
                .setTangent(Math.toRadians(135))
                .afterTime(0.1, new InstantAction(pickupAfterDistance3::enable))
                .splineToSplineHeading(new Pose2d(-59, -45, Math.toRadians(90)), Math.toRadians(90), new TranslationalVelConstraint(40), new ProfileAccelConstraint(-12,22))
                .splineToSplineHeading(new Pose2d(-59, -39, Math.toRadians(90)), Math.toRadians(90), new TranslationalVelConstraint(5), new ProfileAccelConstraint(-12,12))
                .stopAndAdd(new InstantAction(pickupAfterDistance3::failover));


        TrajectoryActionBuilder scoreThirdSamplePath = drive.closeActionBuilder(new Pose2d(-59, -39, Math.toRadians(90)))
                .setReversed(true)
                .splineToLinearHeading(basket, Math.toRadians(225), new TranslationalVelConstraint(50), new ProfileAccelConstraint(-12,30));

        TrajectoryActionBuilder fourthSamplePath = scoreThirdSamplePath.endTrajectory().fresh()
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(-48.5, -33.25, Math.toRadians(180)), Math.toRadians(90), new TranslationalVelConstraint(40), pickSampleAccel)
                .afterTime(0.1, new InstantAction(pickupAfterDistance4::enable))
                .splineToConstantHeading(new Vector2d(-60,-28), Math.toRadians(180), new TranslationalVelConstraint(10), new ProfileAccelConstraint(-10,22))
                .stopAndAdd(new InstantAction(pickupAfterDistance4::failover));
        TrajectoryActionBuilder scoreFourthSamplePath = drive.closeActionBuilder(new Pose2d(-58, -28, Math.toRadians(180)))
                .setReversed(true)
                .splineToLinearHeading(basket, Math.toRadians(225), new TranslationalVelConstraint(40), new ProfileAccelConstraint(-12,22));

        TrajectoryActionBuilder parkPath = scoreFourthSamplePath.endTrajectory().fresh()
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-50, -59), Math.toRadians(45));

        FailoverAction firstSample = new FailoverAction(firstSamplePath.build(), new InstantAction(() -> drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0))));
        FailoverAction secondSample = new FailoverAction( secondSamplePath.build(), new InstantAction(() -> drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0))));
        Action scoreSecondSample = scoreSecondSamplePath.build();
        FailoverAction thirdSample = new FailoverAction(thirdSamplePath.build(), new InstantAction(() -> drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0))));
        Action scoreThirdSample = scoreThirdSamplePath.build();
        Action fourthSample = fourthSamplePath.build();
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
                                goToSample(secondSample, pickupAfterDistance2),
                                collectSampleAndScore(scoreSecondSample, RobotConstants.claw_floor_pickup),
                                goToSample(thirdSample, pickupAfterDistance3),
                                intake.grab(RobotConstants.claw_closed),
                                new ParallelAction(
                                        intake.raiseArm(),
                                        scoreThirdSample
                                ),
                                intake.score(),
                                new InstantAction(() -> intake.moveClaw(RobotConstants.claw_open)),
                                intake.armAction(500,800),
                                new InstantAction(() -> intake.moveClaw(RobotConstants.claw_open)),
                                new ParallelAction(fourthSample, intake.armAction(500),
                                        new InstantAction(() -> intake.distance.setOn(true)),
                                        new SequentialAction(intake.slideAction(200),
                                                new InstantAction(() -> intake.turnAndRotateClaw(180,90))
                                        )
                                ),
                                new InstantAction(() -> intake.arm.setExitWithTime(true)),
                                intake.armAction(50),
                                new InstantAction(() -> intake.arm.setExitWithTime(false)),
                                intake.grab(RobotConstants.claw_closed),
                                new ParallelAction(
                                        intake.raiseArmButNotSnagOnBasket(),
                                        scoreFourthSample,
                                        new SequentialAction(
                                                pickupAfterDistance4,
                                                new InstantAction(thirdSample::failover)
                                        )
                                ),
                                intake.score(true)
                        )
                )
        );
        //Arm to 100
        //Slides to 650
        //Wrist to 0.45


    }
}