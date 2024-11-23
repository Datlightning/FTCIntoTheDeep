package org.firstinspires.ftc.teamcode.opmodes;

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
        initAuto();
        Pose2d beginPose = new Pose2d(-7.125, -64, Math.toRadians(180));
        drive = new MecanumDrive(hardwareMap, beginPose);
        drive.mountTrafficLight(trafficLight);
        rear_distance = new Distance(hardwareMap, telemetry, RobotConstants.rear_distance, timer);

        TrajectoryActionBuilder scoreSpecimenPath = drive.actionBuilder(beginPose)
                .afterTime(0.5, new InstantAction(() -> intake.moveWrist(RobotConstants.specimen_deliver)))
                .setReversed(true)
                .strafeTo(new Vector2d(-7.125, -58))
                .splineToSplineHeading(new Pose2d(0, -36, Math.toRadians(270)), Math.toRadians(90), new TranslationalVelConstraint(40), new ProfileAccelConstraint(-10,30));

        TrajectoryActionBuilder firstSamplePath = drive.actionBuilder(new Pose2d(0, -39, Math.toRadians(270)))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-7.125, -40), Math.toRadians(180))
                .afterTime(0.5,  new InstantAction(() -> intake.moveWrist(RobotConstants.floor_pickup_position)))
                .splineToSplineHeading(new Pose2d(-47.5, -39, Math.toRadians(90)), Math.toRadians(90), new TranslationalVelConstraint(50), new ProfileAccelConstraint(-10,30));

        TrajectoryActionBuilder scoreFirstSamplePath = firstSamplePath.endTrajectory().fresh()
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-50, -51, Math.toRadians(45)), Math.toRadians(225));

        TrajectoryActionBuilder secondSamplePath = scoreFirstSamplePath.endTrajectory().fresh()
                .setReversed(false)
                .stopAndAdd( new InstantAction(() -> intake.moveWrist(RobotConstants.floor_pickup_position)))
                .splineToLinearHeading(new Pose2d(-57.5, -37.5, Math.toRadians(90)), Math.toRadians(90), new TranslationalVelConstraint(30), new ProfileAccelConstraint(-12,22));


        TrajectoryActionBuilder scoreSecondSamplePath = secondSamplePath.endTrajectory().fresh()
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-50, -51, Math.toRadians(45)), Math.toRadians(225));
        TrajectoryActionBuilder thirdSamplePath = scoreSecondSamplePath.endTrajectory().fresh()
                .setReversed(false)
                .afterTime(0.5, new InstantAction(() -> intake.moveWrist(RobotConstants.floor_pickup_position)))
                .splineToSplineHeading(new Pose2d(-51.5,-38.5, Math.toRadians(180)), Math.toRadians(90), new TranslationalVelConstraint(30), new ProfileAccelConstraint(-12,22))
                .splineToConstantHeading(new Vector2d(-51.5,-24), Math.toRadians(270), new TranslationalVelConstraint(10), new ProfileAccelConstraint(-15,20))
                .splineToConstantHeading(new Vector2d(-56.5,-24), Math.toRadians(180), new TranslationalVelConstraint(10), new ProfileAccelConstraint(-10,15));
        TrajectoryActionBuilder scoreThirdSamplePath = thirdSamplePath.endTrajectory().fresh()
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-50, -51, Math.toRadians(45)), Math.toRadians(225), new TranslationalVelConstraint(40), new ProfileAccelConstraint(-22,22));
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

        waitForStart();


        Actions.runBlocking(
                new ParallelAction(
                        trafficLight.updateAction(),
                        intake.updateAction(),
                        rear_distance.updateAction(),
                        new SequentialAction(
                                new ParallelAction(
                                        scoreSpecimen,
                                        raiseArmForSpecimen()
                                ),
                                scoreSpecimen(rear_distance),
                                new SequentialAction(
                                    intake.slideAction(0),
                                       new ParallelAction(
                                               intake.armAction(0),
                                               firstSample
                                       )
                                ),

                                collectSampleAndScore(scoreFirstSample, 0.73),
                                new ParallelAction(
                                        new SequentialAction(intake.armAction(0, 800), secondSample),

                                        intake.armAction(0)
                                ),
                                collectSampleAndScore(scoreSecondSample, 0.88),
                                intake.armAction(0,800),
                                new ParallelAction(thirdSample, intake.armAction(0), intake.slideAction(0)),
                                collectSampleAndScore(scoreThirdSample, 0.73),

                                intake.armAction(0)
                        )
                )
        );
        //Arm to 100
        //Slides to 650
        //Wrist to 0.45


    }
}