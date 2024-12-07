package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.RobotConstants.ARM_LIMIT;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Distance;

@Autonomous
public class Auto1Specimen extends NGAutoOpMode {
    Distance rear_distance;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(-9, -64, Math.toRadians(180));
        initAuto(beginPose);
        rear_distance = new Distance(hardwareMap, telemetry, RobotConstants.rear_distance, timer);

        TrajectoryActionBuilder scoreSpecimenPath = drive.actionBuilder(beginPose)
                .afterTime(0.5, new InstantAction(() -> intake.moveWrist(RobotConstants.specimen_deliver)))
                .setReversed(true)
                .strafeTo(new Vector2d(-9, -58))
                .splineToSplineHeading(new Pose2d(-6, -32, Math.toRadians(270)), Math.toRadians(90), new TranslationalVelConstraint(40), new ProfileAccelConstraint(-10,30));

        TrajectoryActionBuilder firstSamplePath = scoreSpecimenPath.endTrajectory().fresh()
                .setReversed(false)
//                .afterDisp(7.2, new InstantAction(() -> intake.openClaw()))
                .splineTo(new Vector2d(-6, -43), Math.toRadians(270))
                .stopAndAdd( new InstantAction(() -> intake.moveWrist(RobotConstants.floor_pickup_position)))
                .afterTime(0.1, intake.slideAction(0))
                .afterTime(0.1, intake.armAction(0))
                .stopAndAdd( new InstantAction(() -> intake.moveClaw(RobotConstants.claw_floor_pickup)))
                .splineTo(new Vector2d(-48, -40), Math.toRadians(90), new TranslationalVelConstraint(50), new ProfileAccelConstraint(-10,50));



        TrajectoryActionBuilder scoreFirstSamplePath = firstSamplePath.endTrajectory().fresh()
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-55, -58, Math.toRadians(45)), Math.toRadians(225));

        TrajectoryActionBuilder secondSamplePath = scoreFirstSamplePath.endTrajectory().fresh()
                .setReversed(false)
                .stopAndAdd( new InstantAction(() -> intake.moveWrist(RobotConstants.floor_pickup_position)))
                .splineToLinearHeading(new Pose2d(-61, -40, Math.toRadians(90)), Math.toRadians(90), new TranslationalVelConstraint(40), new ProfileAccelConstraint(-12,35));


        TrajectoryActionBuilder scoreSecondSamplePath = secondSamplePath.endTrajectory().fresh()
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-55, -58, Math.toRadians(45)), Math.toRadians(225));

        TrajectoryActionBuilder thirdSamplePath = scoreSecondSamplePath.endTrajectory().fresh()
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(-50,-32, Math.toRadians(180)), Math.toRadians(90), new TranslationalVelConstraint(40), new ProfileAccelConstraint(-16,30))
                .splineToConstantHeading(new Vector2d(-53,-29.5), Math.toRadians(180), new TranslationalVelConstraint(25), new ProfileAccelConstraint(-18,20));

        TrajectoryActionBuilder scoreThirdSamplePath = thirdSamplePath.endTrajectory().fresh()
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-56, -58, Math.toRadians(45)), Math.toRadians(225), new TranslationalVelConstraint(50), new ProfileAccelConstraint(-12,28));
        TrajectoryActionBuilder parkPath = scoreThirdSamplePath.endTrajectory().fresh()
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-50, -59), Math.toRadians(45));

        Action scoreSpecimen = scoreSpecimenPath.build();
        Action firstSample = firstSamplePath.build();
        Action scoreFirstSample = scoreFirstSamplePath.build();
        Action secondSample = secondSamplePath.build();
        Action scoreSecondSample = scoreSecondSamplePath.build();
        Action thirdSample = thirdSamplePath.build();
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
                                            intake.armAction(ARM_LIMIT - 100, 100),
                                            new ParallelAction(
                                                    intake.armAction(ARM_LIMIT - 100),
                                                    intake.slideAction(200)
                                            )
                                    ),
                                    scoreSpecimen
                                ),
                                intake.slideAction(0),
                                new ParallelAction(
                                        firstSample,
                                        openClawAfterDistance(7.5, rear_distance)
                                ),

                                collectSampleAndScore(scoreFirstSample, RobotConstants.claw_floor_pickup),
                                goToSample(secondSample),
                                collectSampleAndScore(scoreSecondSample, RobotConstants.claw_open),
                                intake.armAction(400,800),
                                new InstantAction(() -> intake.moveClaw(RobotConstants.claw_open)),
                                new ParallelAction(thirdSample, intake.armAction(400),
                                        new SequentialAction(intake.slideAction(200),
                                                new InstantAction(() -> intake.turnAndRotateClaw(180,90))
                                        )
                                ),
                                drive.moveUsingDistance(intake.distance, 3.7, 3.5, 4.1, 10),
                                new InstantAction(() -> intake.arm.setExitWithTime(true)),
                                intake.armAction(150),
                                new InstantAction(() -> intake.arm.setExitWithTime(false)),
                                intake.grab(RobotConstants.claw_closed),
                                new ParallelAction(
                                        intake.raiseArmButNotSnagOnBasket(),
                                        scoreThirdSample
                                ),
                                intake.score(true)
                        )
                )
        );


    }
}