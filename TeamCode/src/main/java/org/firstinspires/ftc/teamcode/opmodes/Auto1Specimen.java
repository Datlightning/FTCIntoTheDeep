package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.RobotConstants.ARM_LIMIT;
import static org.firstinspires.ftc.teamcode.RobotConstants.distance;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.MinMax;
import com.acmerobotics.roadrunner.NullAction;
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
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Distance;
import org.firstinspires.ftc.teamcode.subsystems.Rigging;

@Autonomous
public class Auto1Specimen extends NGAutoOpMode {
    Distance rear_distance;
    double INCHES_FORWARD = -2;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(-10, -66, Math.toRadians(270));

        Pose2d basket = new Pose2d(-57.5, -57.5 + INCHES_FORWARD, Math.toRadians(45));
        Pose2d sample1Pickup = new Pose2d(-48, -50.5 + INCHES_FORWARD, Math.toRadians(90));
        Pose2d sample2Pickup = new Pose2d(-59, -50.5 + INCHES_FORWARD, Math.toRadians(90));
        Pose2d sample3Pickup = new Pose2d(-58 + INCHES_FORWARD * Math.cos(Math.toRadians(118.5)), -47 + INCHES_FORWARD * Math.sin(Math.toRadians(118.5)), Math.toRadians(118.5));


        initAuto(beginPose);
        rear_distance = new Distance(hardwareMap, telemetry, RobotConstants.rear_distance, timer);
        AccelConstraint pickSampleAccel = (robotPose, _path, _disp) -> {
            if (robotPose.position.y.value() < -42.0) {
                return new MinMax(-13,22);
            } else {
                return new MinMax(-30,60);
            }
        };
        FailoverAction firstDelay = new FailoverAction(new SleepAction(5), new NullAction());
        TrajectoryActionBuilder scoreSpecimenPath = drive.actionBuilder(beginPose)
                .setReversed(true)
                .afterTime(1, new InstantAction(() -> intake.moveWrist(RobotConstants.specimen_deliver)))
                .splineToConstantHeading(new Vector2d(-6,-36), Math.toRadians(90), new TranslationalVelConstraint(40), new ProfileAccelConstraint(-10,30));

        TrajectoryActionBuilder firstSamplePath = scoreSpecimenPath.endTrajectory().fresh()
                .setReversed(false)
                .afterDisp(7.4, new InstantAction(() -> intake.openClaw()))
                .splineTo(new Vector2d(-6, -43), Math.toRadians(270))
                .splineTo(new Vector2d(-46.5, -57), Math.toRadians(135), new TranslationalVelConstraint(50), new ProfileAccelConstraint(-10,50))
                .afterDisp(0.2, new InstantAction(firstDelay::failover))
                .splineTo(sample1Pickup.position, Math.toRadians(90), new TranslationalVelConstraint(50), new ProfileAccelConstraint(-10,50));

        TrajectoryActionBuilder scoreFirstSamplePath = drive.closeActionBuilder(sample1Pickup)
                .setReversed(true)
                .setTangent(Math.PI + Math.atan2(sample1Pickup.position.y - basket.position.y, sample1Pickup.position.x - basket.position.x))
                .splineToLinearHeading(basket, Math.toRadians(225), new TranslationalVelConstraint(36), new ProfileAccelConstraint(-10,40));

        TrajectoryActionBuilder secondSamplePath = scoreFirstSamplePath.endTrajectory().fresh()
                .setReversed(false)
                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(sample2Pickup, Math.toRadians(90), new TranslationalVelConstraint(40),  new ProfileAccelConstraint(-12,32));

        TrajectoryActionBuilder scoreSecondSamplePath = drive.closeActionBuilder(sample2Pickup)
                .setReversed(true)
                .setTangent(Math.PI + Math.atan2(sample2Pickup.position.y - basket.position.y, sample2Pickup.position.x - basket.position.x))
                .splineToLinearHeading(basket, Math.toRadians(225), new TranslationalVelConstraint(36), new ProfileAccelConstraint(-10,40));

        TrajectoryActionBuilder thirdSamplePath = scoreSecondSamplePath.endTrajectory().fresh()
                .setReversed(false)
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(sample3Pickup, Math.toRadians(90), new TranslationalVelConstraint(40),  new ProfileAccelConstraint(-12,32));
        TrajectoryActionBuilder scoreThirdSamplePath = drive.actionBuilder(sample3Pickup)
                .setReversed(true)
                .setTangent(Math.toRadians(298.5))
                .splineToLinearHeading(basket, Math.toRadians(225), new TranslationalVelConstraint(36), new ProfileAccelConstraint(-10,40));

        TrajectoryActionBuilder parkPath = scoreThirdSamplePath.endTrajectory().fresh()
                .setReversed(true)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-32, -12, Math.toRadians(0)), Math.toRadians(0), new TranslationalVelConstraint(80));


        FailoverAction scoreSpecimen = new FailoverAction(scoreSpecimenPath.build(), new InstantAction(() -> drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0))));
        FailoverAction firstSample = new FailoverAction(firstSamplePath.build(), new InstantAction(() -> drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0))));
        Action scoreFirstSample = scoreFirstSamplePath.build();
        FailoverAction secondSample = new FailoverAction( secondSamplePath.build(), new InstantAction(() -> drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0))));
        Action scoreSecondSample = scoreSecondSamplePath.build();
        FailoverAction thirdSample = new FailoverAction(thirdSamplePath.build(), new InstantAction(() -> drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0))));
        Action scoreThirdSample = scoreThirdSamplePath.build();
        Action park = parkPath.build();
        rear_distance.setOn(true);
        waitForStart();


        Actions.runBlocking(
                new ParallelAction(
                        intake.distance.updateAction(),
                        rear_distance.updateAction(),
                        intake.updateAction(),
                        trafficLight.updateAction(),
                        new SequentialAction(
                                new ParallelAction(
                                        new SequentialAction(
                                            intake.armAction(ARM_LIMIT, 100),
                                            new ParallelAction(
                                                    intake.armAction(ARM_LIMIT),
                                                    intake.slideAction(100)
                                            )
                                    ),
                                    afterDistance(1, rear_distance, new InstantAction(scoreSpecimen::failover)),
                                    scoreSpecimen
                                ),

                                new ParallelAction(
                                        new SequentialAction(
                                                new SleepAction(2),
                                                new InstantAction(() -> intake.moveClaw(RobotConstants.claw_flat))
                                            ),
                                        goToSampleWithSlides(firstSample, firstDelay)
                                ),
                                slideCollectSampleAndScore(scoreFirstSample, RobotConstants.claw_flat),
                                goToSampleWithSlides(secondSample),
                                slideCollectSampleAndScore(scoreSecondSample, RobotConstants.claw_flat),
                                goToSampleWithSlides(thirdSample),
                                slideCollectSampleAndScore(scoreThirdSample, RobotConstants.claw_open),
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


    }
}