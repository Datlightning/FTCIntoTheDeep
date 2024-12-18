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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.TrafficLight;
@Autonomous
public class Auto4Sample extends NGAutoOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(-32.5, -64, Math.toRadians(0));
        initAuto(beginPose);

        Pose2d basket = new Pose2d(-55, -55, Math.toRadians(45));
        TrajectoryActionBuilder scoreSpecimenPath = drive.actionBuilder(beginPose)
                .setReversed(true)
                .afterTime(0.5, new InstantAction(() -> intake.moveWrist(RobotConstants.floor_pickup_position)))
                .setTangent(Math.toRadians(135))
                .splineToConstantHeading(new Vector2d(-39, -60), Math.toRadians(135))
                .splineToSplineHeading(basket, Math.toRadians(135), new TranslationalVelConstraint(40), new ProfileAccelConstraint(-12,40));

        TrajectoryActionBuilder firstSamplePath = scoreSpecimenPath.endTrajectory().fresh()
                .setReversed(false)
                .stopAndAdd( new InstantAction(() -> intake.moveWrist(RobotConstants.floor_pickup_position)))
                .splineToSplineHeading(new Pose2d(-48, -43, Math.toRadians(90)), Math.toRadians(90), new TranslationalVelConstraint(40), new ProfileAccelConstraint(-18,40))
                .splineToSplineHeading(new Pose2d(-48, -39, Math.toRadians(90)), Math.toRadians(90), new TranslationalVelConstraint(40), new ProfileAccelConstraint(-18,40));



        TrajectoryActionBuilder scoreFirstSamplePath = firstSamplePath.endTrajectory().fresh()
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-55, -57, Math.toRadians(45)), Math.toRadians(225), new TranslationalVelConstraint(50), new ProfileAccelConstraint(-12,30));

        TrajectoryActionBuilder secondSamplePath = scoreFirstSamplePath.endTrajectory().fresh()
                .setReversed(false)
                .stopAndAdd( new InstantAction(() -> intake.moveWrist(RobotConstants.floor_pickup_position)))
                .setTangent(Math.toRadians(135))
                .splineToSplineHeading(new Pose2d(-59, -45, Math.toRadians(90)), Math.toRadians(90), new TranslationalVelConstraint(40), new ProfileAccelConstraint(-12,22))
                .splineToSplineHeading(new Pose2d(-59, -39, Math.toRadians(90)), Math.toRadians(90), new TranslationalVelConstraint(5), new ProfileAccelConstraint(-12,12));


        TrajectoryActionBuilder scoreSecondSamplePath = secondSamplePath.endTrajectory().fresh()
                .setReversed(true)
                .splineToLinearHeading(basket, Math.toRadians(225), new TranslationalVelConstraint(50), new ProfileAccelConstraint(-12,30));

        TrajectoryActionBuilder thirdSamplePath = scoreSecondSamplePath.endTrajectory().fresh()
                .setReversed(false)
                .splineTo(new Vector2d(-48,-33), Math.toRadians(135), new TranslationalVelConstraint(30), new ProfileAccelConstraint(-15,22))
                .splineTo(new Vector2d(-58,-28), Math.toRadians(180), new TranslationalVelConstraint(10), new ProfileAccelConstraint(-10,12));

        TrajectoryActionBuilder scoreThirdSamplePath = thirdSamplePath.endTrajectory().fresh()
                .setReversed(true)
                .splineToLinearHeading(basket, Math.toRadians(225), new TranslationalVelConstraint(40), new ProfileAccelConstraint(-12,22));
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
                        intake.distance.updateAction(),
                        intake.updateAction(),
                        trafficLight.updateAction(),
                        new SequentialAction(
                                new ParallelAction(
                                scoreSpecimen,
                                new SequentialAction(
                                        new SleepAction(0.3),
                                        intake.raiseArm(false)
                                )
                                        ),
                                intake.score(),
                                goToSample(firstSample),
                                collectSampleAndScore(scoreFirstSample, RobotConstants.claw_floor_pickup),
                                goToSample(secondSample),
                                intake.grab(RobotConstants.claw_closed),
                                new ParallelAction(
                                        intake.raiseArm(),
                                        scoreSecondSample
                                ),
                                intake.score(),
                                new InstantAction(() -> intake.moveClaw(RobotConstants.claw_open)),
                                intake.armAction(500,800),
                                new InstantAction(() -> intake.moveClaw(RobotConstants.claw_open)),
                                new ParallelAction(thirdSample, intake.armAction(500),
                                        new SequentialAction(intake.slideAction(200),
                                                new InstantAction(() -> intake.turnAndRotateClaw(180,90))
                                        )
                                ),
                                drive.moveUsingDistance(intake.distance, 3.3, 2.7, 3.7, 10),
                                new InstantAction(() -> intake.arm.setExitWithTime(true)),
                                intake.armAction(50),
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
        //Arm to 100
        //Slides to 650
        //Wrist to 0.45


    }
}