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

        Pose2d beginPose = new Pose2d(-34, -64, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        drive.mountTrafficLight(trafficLight);


        TrajectoryActionBuilder scoreSpecimenPath = drive.actionBuilder(beginPose)
                .setReversed(true)
                .afterTime(0.5, new InstantAction(() -> intake.moveWrist(RobotConstants.floor_pickup_position)))
                .splineToConstantHeading(new Vector2d(-39, -60), Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(-48, -51, Math.toRadians(45)), Math.toRadians(135), new TranslationalVelConstraint(40), new ProfileAccelConstraint(-22,40));
        TrajectoryActionBuilder firstSamplePath = scoreSpecimenPath.endTrajectory().fresh()
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(-49, -39, Math.toRadians(90)), Math.toRadians(90), new TranslationalVelConstraint(40), new ProfileAccelConstraint(-18,40));



        TrajectoryActionBuilder scoreFirstSamplePath = firstSamplePath.endTrajectory().fresh()
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-48, -53, Math.toRadians(45)), Math.toRadians(225));

        TrajectoryActionBuilder secondSamplePath = scoreFirstSamplePath.endTrajectory().fresh()
                .setReversed(false)
                .stopAndAdd( new InstantAction(() -> intake.moveWrist(RobotConstants.floor_pickup_position)))
                .splineToLinearHeading(new Pose2d(-59, -37.5, Math.toRadians(90)), Math.toRadians(90), new TranslationalVelConstraint(30), new ProfileAccelConstraint(-12,22));


        TrajectoryActionBuilder scoreSecondSamplePath = secondSamplePath.endTrajectory().fresh()
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-47, -51, Math.toRadians(45)), Math.toRadians(225));
        TrajectoryActionBuilder thirdSamplePath = scoreSecondSamplePath.endTrajectory().fresh()
                .setReversed(false)
                .afterTime(0.5, new InstantAction(() -> intake.moveWrist(RobotConstants.floor_pickup_position)))
                .splineToSplineHeading(new Pose2d(-53,-38.5, Math.toRadians(180)), Math.toRadians(90), new TranslationalVelConstraint(30), new ProfileAccelConstraint(-12,22))
                .splineToConstantHeading(new Vector2d(-53,-25), Math.toRadians(270), new TranslationalVelConstraint(10), new ProfileAccelConstraint(-15,20))
                .splineToConstantHeading(new Vector2d(-58,-25), Math.toRadians(180), new TranslationalVelConstraint(10), new ProfileAccelConstraint(-10,15));
        TrajectoryActionBuilder scoreThirdSamplePath = thirdSamplePath.endTrajectory().fresh()
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-46, -53, Math.toRadians(45)), Math.toRadians(225), new TranslationalVelConstraint(40), new ProfileAccelConstraint(-22,22));
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

        intake.moveArm(0);
        while(intake.arm.isBusy()){
            intake.update();
        }
        telemetry.addLine("Arm Lowered, ready to go");
        telemetry.update();

        waitForStart();


        Actions.runBlocking(
                new ParallelAction(
                        intake.updateAction(),
                        trafficLight.updateAction(),
                        new SequentialAction(
                                new ParallelAction(
                                        scoreSpecimen,
                                        intake.raiseArm()
                                ),
                                intake.score(),
                                new InstantAction(() -> intake.moveClaw(0.73)),
                                new ParallelAction(
                                        new SequentialAction(intake.armAction(0, 800), firstSample),
                                        intake.armAction(0),
                                        intake.slideAction(0)
                                ),
                                drive.moveUsingDistance(intake.distance, RobotConstants.TARGET, RobotConstants.TOO_CLOSE, RobotConstants.TOO_FAR, RobotConstants.GIVE_UP),
                                new InstantAction(() -> intake.moveWrist(RobotConstants.floor_pickup_position)),
                                new SleepAction(0.6),
                                intake.grab(1),
                                new ParallelAction(
                                        intake.raiseArm(),
                                        scoreFirstSample
                                ),
                                intake.score(),
                                new InstantAction(() -> intake.moveClaw(0.73)),
                                new ParallelAction(
                                        new SequentialAction(intake.armAction(0, 800), secondSample),

                                        intake.armAction(0)
                                        ),
                                drive.moveUsingDistance(intake.distance, RobotConstants.TARGET, RobotConstants.TOO_CLOSE, RobotConstants.TOO_FAR, RobotConstants.GIVE_UP),
                                intake.grab(1),
                                new ParallelAction(
                                        intake.raiseArm(),
                                        scoreSecondSample
                                ),
                                intake.score(),
                                new InstantAction(() -> intake.moveClaw(0.88)),
                                intake.armAction(0,800),
                                new ParallelAction(thirdSample, intake.armAction(0), intake.slideAction(0)),
                                drive.moveUsingDistance(intake.distance, RobotConstants.TARGET, RobotConstants.TOO_CLOSE, RobotConstants.TOO_FAR, RobotConstants.GIVE_UP),
                                new SleepAction(0.3),
                                intake.grab(1),
                                new ParallelAction(
                                        intake.raiseArm(),
                                        scoreThirdSample
                                ),
                                intake.scoreLast(),
                                intake.armAction(0)
                        )
                )
        );
        //Arm to 100
        //Slides to 650
        //Wrist to 0.45


    }
}