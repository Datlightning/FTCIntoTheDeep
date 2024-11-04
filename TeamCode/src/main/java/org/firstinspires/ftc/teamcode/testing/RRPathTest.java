package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.InstantFunction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.TankDrive;
import org.firstinspires.ftc.teamcode.roadrunner.tuning.TuningOpModes;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

import java.util.function.DoubleSupplier;

@Autonomous
public class RRPathTest extends LinearOpMode {
    Intake intake;
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(-10, -63, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        intake = new Intake(hardwareMap, telemetry);
        intake.init();

        intake.calculateOffset();
        intake.moveClaw(0.95);
        waitForStart();
        TrajectoryActionBuilder scoreSpecimenPath = drive.actionBuilder(beginPose)
                .setReversed(true)
                .afterTime(0.5, new InstantAction(() -> intake.moveWrist(0.6)))
                .splineToLinearHeading(new Pose2d(-6, -34, Math.toRadians(270)), Math.toRadians(90))
//                .stopAndAdd(new InstantAction(() -> intake.moveWrist(0.7) ))
                .waitSeconds(0.6);
//                .lineToY(-40);
        TrajectoryActionBuilder firstSamplePath = scoreSpecimenPath.endTrajectory().fresh()
                .waitSeconds(0.2)
                .setReversed(false)
                .afterTime(0.5, new InstantAction(() -> intake.moveWrist(RobotConstants.floor_pickup_position)))
                .splineToLinearHeading(new Pose2d(-40, -42, Math.toRadians(90)), Math.toRadians(90));



        TrajectoryActionBuilder scoreFirstSamplePath = firstSamplePath.endTrajectory().fresh()
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-45, -57, Math.toRadians(45)), Math.toRadians(225));

        TrajectoryActionBuilder secondSamplePath = scoreFirstSamplePath.endTrajectory().fresh()
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(-50, -42, Math.toRadians(90)), Math.toRadians(90));


        TrajectoryActionBuilder scoreSecondSamplePath = secondSamplePath.endTrajectory().fresh()
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-43, -55, Math.toRadians(45)), Math.toRadians(225));
        TrajectoryActionBuilder thirdSamplePath = scoreSecondSamplePath.endTrajectory().fresh()
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(-45,-40, Math.toRadians(180)), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-45,-24), Math.toRadians(180));


        Action scoreSpecimen = scoreSpecimenPath.build();
        Action firstSample = firstSamplePath.build();
        Action scoreFirstSample = scoreFirstSamplePath.build();
        Action secondSample = secondSamplePath.build();
        Action scoreSecondSample = scoreSecondSamplePath.build();
        Action thirdSample = thirdSamplePath.build();
//        Action scoreAction = new SequentialAction(
//                intake.armAction(1450),
//                new InstantAction(() -> intake.openClaw()),
//                new SleepAction(0.1),
//                new InstantAction(() -> intake.moveWrist(RobotConstants.floor_pickup_position)),
//                new SleepAction(0.3),
//                intake.slideAction(0)
//        );
//        Action raiseArm = new SequentialAction(
//                intake.armAction(1400, 600),
//                new ParallelAction(
////                        intake.slideAction(slide_extension),
//                        intake.armAction(1400)
//                ),
//                new InstantAction(() -> intake.moveWrist(0.9))
//        );
        Actions.runBlocking(
                new ParallelAction(
                    intake.updateAction(),
                    new SequentialAction(
                            new ParallelAction(
                                    scoreSpecimen,
                                    intake.armAction(1550)
                            ),
                            new SleepAction(0.3),
                            new InstantAction(() -> intake.moveClaw(0.7)),
                            new ParallelAction(
                                    firstSample,
                                    intake.armAction(0)
                            ),
                            drive.actionBuilder(drive.pose)
                                    .splineTo(new Vector2d(drive.pose.position.x,
                                                    drive.pose.position.y + ((DoubleSupplier) () -> intake.distance.getDist()).getAsDouble() - RobotConstants.TOO_CLOSE + 0.5)
                                                    , Math.toRadians(90)).
                                    build(),
                            intake.grab(),
                            new ParallelAction(
                                    intake.raiseArm(),
                                    scoreFirstSample
                            ),
                            intake.score(),
                            new ParallelAction(
                                    intake.armAction(0),
                                    secondSample
                            ),
                            drive.actionBuilder(drive.pose)
                                    .splineTo(new Vector2d(drive.pose.position.x, drive.pose.position.y  - RobotConstants.TOO_CLOSE + 0.5), Math.toRadians(90)).
                                    build(),
                            intake.grab(),
                            new ParallelAction(
                                intake.raiseArm(),
                                scoreSecondSample
                            ),
                            intake.score(),
                            new ParallelAction(thirdSample, intake.armAction(0), intake.slideAction(0))//TODO: fix
                )));
        //Arm to 100
        //Slides to 650
        //Wrist to 0.45


    }
}