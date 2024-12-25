package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.RobotConstants.ARM_LIMIT;
import static org.firstinspires.ftc.teamcode.RobotConstants.distance;

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
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Distance;
import org.firstinspires.ftc.teamcode.subsystems.Rigging;

@Autonomous
public class Auto1Specimen extends NGAutoOpMode {
    Distance rear_distance;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(-10, -66, Math.toRadians(270));
        initAuto(beginPose);
        rear_distance = new Distance(hardwareMap, telemetry, RobotConstants.rear_distance, timer);
        AccelConstraint pickSampleAccel = (robotPose, _path, _disp) -> {
            if (robotPose.position.y.value() < -42.0) {
                return new MinMax(-13,22);
            } else {
                return new MinMax(-30,60);
            }
        };
        TrajectoryActionBuilder scoreSpecimenPath = drive.actionBuilder(beginPose)
                .setReversed(true)
                .afterTime(1, new InstantAction(() -> intake.moveWrist(RobotConstants.specimen_deliver)))
                .splineToConstantHeading(new Vector2d(-6,-35.5), Math.toRadians(90), new TranslationalVelConstraint(40), new ProfileAccelConstraint(-10,30));

        FailoverAction pickupAfterDistance1 = new FailoverAction(pickupAfterDistance(RobotConstants.TOO_FAR, intake.distance), new InstantAction(() -> intake.distance.setOn(false)), false);

        FailoverAction pickupAfterDistance2 = new FailoverAction(pickupAfterDistance(RobotConstants.TOO_FAR , intake.distance), new InstantAction(() -> intake.distance.setOn(false)), false);

        FailoverAction pickupAfterDistance3 = new FailoverAction(intake.distance.waitAction(3.7),new InstantAction(() -> intake.distance.setOn(false)), false );

        TrajectoryActionBuilder firstSamplePath = scoreSpecimenPath.endTrajectory().fresh()
                .setReversed(false)
//                .afterDisp(7.2, new InstantAction(() -> intake.openClaw()))
                .splineTo(new Vector2d(-6, -43), Math.toRadians(270))
                .stopAndAdd( new InstantAction(() -> intake.moveWrist(RobotConstants.floor_pickup_position)))
                .afterTime(0.1, intake.slideAction(0))
                .afterTime(0.1, intake.armAction(0))
                .stopAndAdd( new InstantAction(() -> intake.moveClaw(RobotConstants.claw_floor_pickup)))
                .splineTo(new Vector2d(-48, -44), Math.toRadians(90), new TranslationalVelConstraint(60), new ProfileAccelConstraint(-10,50))
                .afterTime(0.1, new InstantAction(pickupAfterDistance1::enable))
                .splineTo(new Vector2d(-48, -40), Math.toRadians(90), new TranslationalVelConstraint(50), new ProfileAccelConstraint(-10,50))
                .stopAndAdd(new InstantAction(pickupAfterDistance1::failover));

        TrajectoryActionBuilder scoreFirstSamplePath = drive.closeActionBuilder(new Pose2d(-48,-43, Math.toRadians(90)))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-58, -62, Math.toRadians(45)), Math.toRadians(225), new TranslationalVelConstraint(50), new ProfileAccelConstraint(-12,30));

        TrajectoryActionBuilder secondSamplePath = scoreFirstSamplePath.endTrajectory().fresh()
                .setReversed(false)
                .stopAndAdd( new InstantAction(() -> intake.moveWrist(RobotConstants.floor_pickup_position)))
                .setTangent(Math.toRadians(135))
                .afterTime(0.1, new InstantAction(pickupAfterDistance2::enable))
                .splineToSplineHeading(new Pose2d(-60, -44, Math.toRadians(90)), Math.toRadians(90), new TranslationalVelConstraint(40), new ProfileAccelConstraint(-12,22))
                .splineToSplineHeading(new Pose2d(-60, -39, Math.toRadians(90)), Math.toRadians(90), new TranslationalVelConstraint(30), new ProfileAccelConstraint(-8,10))
                .stopAndAdd(new InstantAction(pickupAfterDistance2::failover));

        TrajectoryActionBuilder scoreSecondSamplePath = drive.closeActionBuilder(new Pose2d(-60,-42, Math.toRadians(90)))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-58, -62, Math.toRadians(45)), Math.toRadians(225), new TranslationalVelConstraint(50), new ProfileAccelConstraint(-12,30));

        TrajectoryActionBuilder thirdSamplePath = scoreSecondSamplePath.endTrajectory().fresh()
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(-48.5, -38.25, Math.toRadians(180)), Math.toRadians(90), new TranslationalVelConstraint(40), pickSampleAccel)
                .afterTime(0.1, new InstantAction(pickupAfterDistance3::enable))
                .splineToConstantHeading(new Vector2d(-60,-32), Math.toRadians(180), new TranslationalVelConstraint(10), new ProfileAccelConstraint(-10,22))
                .stopAndAdd(new InstantAction(pickupAfterDistance3::failover));
        TrajectoryActionBuilder scoreThirdSamplePath = thirdSamplePath.endTrajectory().fresh()
                .setReversed(true)
                .setTangent(Math.toRadians(270))
                .splineToSplineHeading(new Pose2d(-52, -50, Math.toRadians(45)), Math.toRadians(270), new TranslationalVelConstraint(30), new ProfileAccelConstraint(-12,32))
                .splineToConstantHeading(new Vector2d(-58, -62), Math.toRadians(225), new TranslationalVelConstraint(10), new ProfileAccelConstraint(-12,32));

        TrajectoryActionBuilder parkPath = scoreThirdSamplePath.endTrajectory().fresh()
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-50, -59), Math.toRadians(45));

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
                                    afterDistance(4.5, rear_distance, new InstantAction(scoreSpecimen::failover)),
                                    scoreSpecimen
                                ),
                                intake.slideAction(0),
                                new ParallelAction(
                                        firstSample,
                                        new SequentialAction(
                                            pickupAfterDistance1,
                                            new InstantAction(firstSample::failover)
                                        ),
                                        openClawAfterDistance(8.5, rear_distance)
                                ),
                                collectSampleAndScore(scoreFirstSample, RobotConstants.claw_floor_pickup, false),
                                goToSample(secondSample, pickupAfterDistance2),
                                collectSampleAndScore(scoreSecondSample, RobotConstants.claw_open, false),
                                intake.armAction(500,1000),
                                new InstantAction(() -> intake.moveClaw(RobotConstants.claw_open )),
                                new ParallelAction(
                                        thirdSample,
                                        intake.armAction(500),
                                        new InstantAction(() -> intake.distance.setOn(true)),
                                        new SequentialAction(
                                                intake.slideAction(200),
                                                new InstantAction(() -> intake.turnAndRotateClaw(180,90))
                                        ),
                                        new SequentialAction(
                                                pickupAfterDistance3,
                                                new InstantAction(thirdSample::failover)
                                        )

                                ),
//                                drive.moveUsingDistance(intake.distance, 3.3, 2.7, 3.7, 10),
//                                telemetryLine("HERE"),
//                                new SleepAction(30),
                                new InstantAction(() -> intake.arm.setExitWithTime(true)),
                                intake.armAction(50),
                                new InstantAction(() -> intake.arm.setExitWithTime(false)),
                                intake.grab(RobotConstants.claw_closed),
                                new ParallelAction(
                                        intake.raiseArmButNotSnagOnBasket(),
                                        scoreThirdSample
                                ),
                                intake.score(true),
                                intake.armAction(0)
                        )
                )
        );


    }
}