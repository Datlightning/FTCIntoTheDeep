package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.RobotConstants.ARM_LIMIT;

import com.acmerobotics.dashboard.config.Config;
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
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystems.Distance;

@Config
@Autonomous
public class Auto3Specimen extends NGAutoOpMode {
    Distance rear_distance;
    public static int SPECIMEN_COUNT = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(10, -64, Math.toRadians(90));
        initAuto(beginPose);

        rear_distance = new Distance(hardwareMap, telemetry, RobotConstants.rear_distance, timer);
        AccelConstraint highMode = (robotPose, _path, _disp) -> {
            if (robotPose.position.y.value() < -20.0) {
                return new MinMax(-10,20);
            } else {
                return new MinMax(-50,80);
            }
        };
        VelConstraint highModeVel = (robotPose, _path, _disp) -> {
            if (robotPose.position.y.value() < -25.0) {
                return 30;
            } else {
                return 65;
            }
        };

        AccelConstraint smartScore = (robotPose, _path, _disp) -> {
            if (robotPose.position.y.value() < -32.0) {
                return new MinMax(-15,40);
            } else {
                return new MinMax(-30,50);
            }
        };





        double CHAMBER_Y = -35.5;
        Pose2d firstSample = new Pose2d(23, -34, Math.toRadians(15));
        Pose2d secondSample = new Pose2d(58, -14, Math.toRadians(270));
        Pose2d thirdSample = new Pose2d(58, -14, Math.toRadians(315));
        Pose2d pickupPosition = new Pose2d(40, -63,  Math.toRadians(270));
        TrajectoryActionBuilder scoreFirstSpecimenPath = drive.actionBuilder(beginPose)
                .splineToConstantHeading(new Vector2d(4,-58.5), Math.toRadians(90), new TranslationalVelConstraint(30), smartScore)
                .splineToConstantHeading(new Vector2d(4,-47), Math.toRadians(90), new TranslationalVelConstraint(30), smartScore);
        TrajectoryActionBuilder moveToFirstSamplePath = drive.closeActionBuilder(new Pose2d(4, -47, Math.toRadians(90)))
                .setTangent(Math.toRadians(-10))
                .splineToLinearHeading(new Pose2d(23, -34, Math.toRadians(15)), Math.toRadians(45));
        TrajectoryActionBuilder depositFirstSamplePath = moveToFirstSamplePath.endTrajectory().fresh()
                .setTangent(Math.toRadians(270))
                .splineToSplineHeading(new Pose2d(48, -35, Math.toRadians(270)), Math.toRadians(0));
        TrajectoryActionBuilder moveToSecondSamplePath = drive.closeActionBuilder(new Pose2d(48, -35, Math.toRadians(270)))
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(secondSample.position,0);
        TrajectoryActionBuilder depositSecondSamplePath = moveToSecondSamplePath.endTrajectory().fresh()
                .setTangent(Math.toRadians(270))
                .lineToY(-35);
        TrajectoryActionBuilder moveToThirdSamplePath = drive.closeActionBuilder(new Pose2d(secondSample.position.x, -35, Math.toRadians(270)))
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(thirdSample,0);
        TrajectoryActionBuilder depositThirdSamplePath = drive.closeActionBuilder(thirdSample)
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(pickupPosition,Math.toRadians(270) );

        Pose2d secondSpecimen = new Pose2d(6, CHAMBER_Y, Math.toRadians(270));
        Pose2d thirdSpecimen = new Pose2d(8, CHAMBER_Y, Math.toRadians(270));
        Pose2d fourthSpecimen = new Pose2d(10, CHAMBER_Y, Math.toRadians(270));
        Pose2d fifthSpecimen = new Pose2d(12, CHAMBER_Y, Math.toRadians(270));
        TrajectoryActionBuilder scoreSecondSpecimenPath = drive.closeActionBuilder(pickupPosition)
                .strafeTo(secondSpecimen.position);
        TrajectoryActionBuilder collectThirdSpecimenPath = drive.closeActionBuilder(secondSpecimen)
                .strafeTo(pickupPosition.position);
        TrajectoryActionBuilder scoreThirdSpecimenPath = drive.closeActionBuilder(pickupPosition)
                .strafeTo(thirdSpecimen.position);
        TrajectoryActionBuilder collectFourthSpecimenPath = drive.closeActionBuilder(thirdSpecimen)
                .strafeTo(pickupPosition.position);
        TrajectoryActionBuilder scoreFourthSpecimenPath = drive.closeActionBuilder(pickupPosition)
                .strafeTo(fourthSpecimen.position);
        TrajectoryActionBuilder collectFifthSpecimenPath = drive.closeActionBuilder(fourthSpecimen)
                .strafeTo(pickupPosition.position);
        TrajectoryActionBuilder scoreFifthSpecimenPath = drive.closeActionBuilder(pickupPosition)
                .strafeTo(fifthSpecimen.position);
        TrajectoryActionBuilder parkPath = scoreFifthSpecimenPath.endTrajectory().fresh()
                .strafeTo(pickupPosition.position);



        Action firstSpecimen = scoreFirstSpecimenPath.build();
        Action moveToFirstSample = moveToFirstSamplePath.build();
        Action depositFirstSample = depositFirstSamplePath.build();
        Action moveToSecondSample = moveToSecondSamplePath.build();
        Action depositSecondSample = depositSecondSamplePath.build();
        Action moveToThirdSample = moveToThirdSamplePath.build();
        Action depositThirdSample = depositThirdSamplePath.build();
        Action scoreSecondSpecimen = scoreSecondSpecimenPath.build();
        Action collectThirdSpecimen = collectThirdSpecimenPath.build();
        Action scoreThirdSpecimen = scoreThirdSpecimenPath.build();
        Action collectFourthSpecimen = collectFourthSpecimenPath.build();
        Action scoreFourthSpecimen = scoreFourthSpecimenPath.build();
        Action collectFifthSpecimen = collectFifthSpecimenPath.build();
        Action scoreFifthSpecimen = scoreFifthSpecimenPath.build();
        Action park = parkPath.build();


        Action auto;
        SPECIMEN_COUNT = 4;
        auto = new SequentialAction(
                intake.armAction(560, 200),
                new ParallelAction(
                    new SequentialAction(
                    intake.slideAction(1000)
                    ),
                    new InstantAction(() -> intake.moveWrist(20)),
                    firstSpecimen
                ),
                new InstantAction(() -> intake.moveClaw(RobotConstants.claw_flat)),
                new SleepAction(0.3),
                intake.slideAction(200, 400),
            transferSample(moveToFirstSample, depositFirstSample, 90   ),
            transferSample(moveToSecondSample, depositSecondSample),
            transferSample(moveToThirdSample, depositThirdSample, true),
            scoreSpecimen(scoreSecondSpecimen, collectThirdSpecimen),
            scoreSpecimen(scoreThirdSpecimen, collectFourthSpecimen),
            scoreSpecimen(scoreFourthSpecimen, collectFifthSpecimen),
            scoreSpecimen(scoreFifthSpecimen, park)
        );



        telemetry.addLine("paths did the creation");
        telemetry.update();

        waitForStart();


        Actions.runBlocking(
                new ParallelAction(
                        intake.updateAction(),
                        trafficLight.updateAction(),
                        rear_distance.updateAction(),
                        intake.distance.updateAction(),
                        auto
                )
        );
        //Arm to 100
        //Slides to 650
        //Wrist to 0.45


    }
    public Action driveAndGrab(){
        return new SequentialAction(
                drive.moveUsingDistance(intake.distance, RobotConstants.TARGET, RobotConstants.TOO_CLOSE, RobotConstants.TOO_FAR, 12),
                intake.grab(RobotConstants.claw_closed)
            );
    }
}