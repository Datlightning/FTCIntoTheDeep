package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

@Autonomous
public class Auto4Sample extends LinearOpMode {
    Intake intake;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Pose2d beginPose = new Pose2d(-38, -64, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        intake = new Intake(hardwareMap, telemetry);
        intake.init();
        intake.slides.setReachedRange(30);
        intake.calculateOffset();
        intake.moveClaw(1);

        TrajectoryActionBuilder scoreSpecimenPath = drive.actionBuilder(beginPose)
                .setReversed(true)
                .afterTime(0.5, new InstantAction(() -> intake.moveWrist(RobotConstants.floor_pickup_position)))
                .splineToConstantHeading(new Vector2d(-43, -60), Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(-48, -54, Math.toRadians(45)), Math.toRadians(135), new TranslationalVelConstraint(30), new ProfileAccelConstraint(-22,22));
        TrajectoryActionBuilder firstSamplePath = scoreSpecimenPath.endTrajectory().fresh()
                .setReversed(false)
                .stopAndAdd( new InstantAction(() -> intake.moveWrist(RobotConstants.floor_pickup_position)))
                .splineToLinearHeading(new Pose2d(-44, -36, Math.toRadians(90)), Math.toRadians(90));


        TrajectoryActionBuilder scoreFirstSamplePath = firstSamplePath.endTrajectory().fresh()
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-48, -54, Math.toRadians(45)), Math.toRadians(225));

        TrajectoryActionBuilder secondSamplePath = scoreFirstSamplePath.endTrajectory().fresh()
                .setReversed(false)
                .stopAndAdd( new InstantAction(() -> intake.moveWrist(RobotConstants.floor_pickup_position)))
                .splineToLinearHeading(new Pose2d(-54, -37, Math.toRadians(90)), Math.toRadians(90));


        TrajectoryActionBuilder scoreSecondSamplePath = secondSamplePath.endTrajectory().fresh()
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-47, -53, Math.toRadians(45)), Math.toRadians(225));
        TrajectoryActionBuilder thirdSamplePath = scoreSecondSamplePath.endTrajectory().fresh()
                .setReversed(false)
                .afterTime(0.5, new InstantAction(() -> intake.moveWrist(RobotConstants.floor_pickup_position)))
                .splineToSplineHeading(new Pose2d(-45,-24.5, Math.toRadians(180)), Math.toRadians(180));
        TrajectoryActionBuilder scoreThirdSamplePath = thirdSamplePath.endTrajectory().fresh()
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-45, -52, Math.toRadians(45)), Math.toRadians(225));
        TrajectoryActionBuilder parkPath = scoreThirdSamplePath.endTrajectory().fresh()
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-46, -56), Math.toRadians(45));





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
                        intake.updateAction(),
                        new SequentialAction(
                                new ParallelAction(
                                        scoreSpecimen,
                                        intake.raiseArm()
                                ),
                                intake.score(),
                                new InstantAction(() -> intake.moveClaw(0.7)),
                                new ParallelAction(
                                        firstSample,
                                        intake.armAction(0),
                                        intake.slideAction(0)
                                ),
                                intake.grab(),
                                new ParallelAction(
                                        intake.raiseArm(),
                                        scoreFirstSample
                                ),
                                intake.score(),
                                new InstantAction(() -> intake.moveClaw(0.7)),
                                new ParallelAction(
                                        intake.armAction(0),
                                        secondSample
                                ),
//                                drive.moveUsingDistance(intake.distance, 4, 1.25, 4.5),
                                intake.grab(),
                                new ParallelAction(
                                        intake.raiseArm(),
                                        scoreSecondSample
                                ),
                                intake.score(),
                                new InstantAction(() -> intake.moveClaw(0.7)),
                                intake.armAction(0,800),
                                new ParallelAction(thirdSample, intake.armAction(0), intake.slideAction(0)),
                                drive.moveUsingDistance(intake.distance, 4.125, 3.5,5),
                                new InstantAction(() -> intake.moveWrist(RobotConstants.floor_pickup_position)),
                                new SleepAction(0.3),
                                intake.grab(0.9),
                                new ParallelAction(
                                        intake.raiseArm(),
                                        scoreThirdSample
                                ),
                                intake.scoreLast(),
                                intake.armAction(1250, 1350),
                                new ParallelAction(
                                        intake.slideAction(0),
                                        park
                                ),
                                intake.armAction(0)
                        )
                )
        );
        //Arm to 100
        //Slides to 650
        //Wrist to 0.45


    }
}