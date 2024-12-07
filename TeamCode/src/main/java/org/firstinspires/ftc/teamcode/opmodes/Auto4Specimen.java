package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
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
import org.firstinspires.ftc.teamcode.subsystems.Distance;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.TrafficLight;

@Config
@Autonomous
public class Auto4Specimen extends NGAutoOpMode {
    Distance rear_distance;
    public static int SPECIMEN_COUNT = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(10, -64, Math.toRadians(90));
        initAuto(beginPose);

        rear_distance = new Distance(hardwareMap, telemetry, RobotConstants.rear_distance, timer);

        TrajectoryActionBuilder scoreFirstSpecimenPath = drive.actionBuilder(beginPose)
                .setReversed(false)
                .strafeTo(new Vector2d(10, -58))
                .splineToSplineHeading(new Pose2d(6, -32, Math.toRadians(270)), Math.toRadians(90), new TranslationalVelConstraint(40), new ProfileAccelConstraint(-10,30))
        TrajectoryActionBuilder throwFirstSamplePath = scoreFirstSpecimenPath.endTrajectory().fresh()
                .setReversed(true)
                .splineTo(new Vector2d(6, -43), Math.toRadians(0))
                .splineTo(new Vector2d(36, -46), Math.toRadians(0))
                .splineTo(new Vector2d(48, -34.5), Math.toRadians(90));
        TrajectoryActionBuilder throwSecondSamplePath = throwFirstSamplePath.endTrajectory().fresh()
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(52, -40), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(58, -34.5), Math.toRadians(90));
        TrajectoryActionBuilder clearThirdSamplePath = throwSecondSamplePath.endTrajectory().fresh()
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(55, -14, 0), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(55 , -14), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(60, -50), Math.toRadians(270))
                .splineToSplineHeading(new Pose2d(24, -58, Math.toRadians(0)), Math.toRadians(180));

        TrajectoryActionBuilder scoreSecondSpecimenPath = clearThirdSamplePath.endTrajectory().fresh()
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(10, -32, Math.toRadians(270)), Math.toRadians(90));
        TrajectoryActionBuilder pickThirdSpecimenPath = scoreSecondSpecimenPath.endTrajectory().fresh()
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(28, -58, Math.toRadians(0)), Math.toRadians(0));

        TrajectoryActionBuilder scoreThirdSpecimenPath = pickThirdSpecimenPath.endTrajectory().fresh()
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(13, -32, Math.toRadians(270)), Math.toRadians(90));
        TrajectoryActionBuilder pickFourthSpecimenPath = scoreThirdSpecimenPath.endTrajectory().fresh()
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(28, -58, Math.toRadians(0)), Math.toRadians(0));

        TrajectoryActionBuilder scoreFourthSpecimenPath = pickFourthSpecimenPath.endTrajectory().fresh()
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(16, -32, Math.toRadians(270)), Math.toRadians(90));

        Action scoreFirstSpecimen = scoreFirstSpecimenPath.build();
        Action throwFirstSample = throwFirstSamplePath.build();
        Action throwSecondSample = throwSecondSamplePath.build();
        Action clearThirdSample = clearThirdSamplePath.build();
        Action scoreSecondSpecimen = scoreSecondSpecimenPath.build();
        Action pickThirdSpecimen = pickThirdSpecimenPath.build();
        Action scoreThirdSpecimen = scoreThirdSpecimenPath.build();
        Action pickFourthSpecimen = pickFourthSpecimenPath.build();
        Action scoreFourthSpecimen = scoreFourthSpecimenPath.build();

        Action auto;
        if(SPECIMEN_COUNT == 4){
            auto = new SequentialAction(
                    scoreFirstSpecimen,
                    raiseArmForSpecimen(),
                    scoreSpecimen(rear_distance),
                    throwFirstSample,

                    drive.moveUsingDistance(intake.distance, RobotConstants.TARGET, RobotConstants.TOO_CLOSE, RobotConstants.TOO_FAR, RobotConstants.GIVE_UP),
                    intake.yeetSample(),
                    throwSecondSample,

                    drive.moveUsingDistance(intake.distance, RobotConstants.TARGET, RobotConstants.TOO_CLOSE, RobotConstants.TOO_FAR, RobotConstants.GIVE_UP),
                    intake.yeetSample(),
                    clearThirdSample,

                    trafficLight.warnHuman(),
                    drive.moveUsingDistance(intake.distance, RobotConstants.TARGET, RobotConstants.TOO_CLOSE, RobotConstants.TOO_FAR, 12),
                    intake.grab(RobotConstants.claw_closed),
                    new ParallelAction(scoreSecondSpecimen,trafficLight.disable(),raiseArmForSpecimen()),
                    pickThirdSpecimen,
                    trafficLight.warnHuman(),
                    drive.moveUsingDistance(intake.distance, RobotConstants.TARGET, RobotConstants.TOO_CLOSE, RobotConstants.TOO_FAR, 12),
                    intake.grab(RobotConstants.claw_closed),
                    new ParallelAction(scoreThirdSpecimen,trafficLight.disable(),raiseArmForSpecimen()),
                    raiseArmForSpecimen(),
                    scoreSpecimen(rear_distance),
                    pickFourthSpecimen,
                    trafficLight.warnHuman(),
                    drive.moveUsingDistance(intake.distance, RobotConstants.TARGET, RobotConstants.TOO_CLOSE, RobotConstants.TOO_FAR, 12),
                    new ParallelAction(scoreFourthSpecimen,trafficLight.disable()),
                    raiseArmForSpecimen(),
                    scoreSpecimen(rear_distance)

                    );
        }else if(SPECIMEN_COUNT == 3){
            auto = new SequentialAction(scoreFirstSpecimen,
                    raiseArmForSpecimen(),
                    scoreSpecimen(rear_distance),
                    throwFirstSample,
                    drive.moveUsingDistance(intake.distance, 2, RobotConstants.TOO_CLOSE, RobotConstants.TOO_FAR, RobotConstants.GIVE_UP),
                    intake.yeetSample(),
                    throwSecondSample,
                    drive.moveUsingDistance(intake.distance, 2, RobotConstants.TOO_CLOSE, RobotConstants.TOO_FAR, RobotConstants.GIVE_UP),
                    intake.yeetSample(),
                    clearThirdSample,
                    trafficLight.warnHuman(),

                    drive.moveUsingDistance(intake.distance, 2, RobotConstants.TOO_CLOSE, RobotConstants.TOO_FAR, 12),
                    new ParallelAction(scoreSecondSpecimen,trafficLight.disable()),
                    raiseArmForSpecimen(),
                    scoreSpecimen(rear_distance),
                    pickThirdSpecimen,
                    trafficLight.warnHuman(),

                    drive.moveUsingDistance(intake.distance, 2, RobotConstants.TOO_CLOSE, RobotConstants.TOO_FAR, 12),
                    new ParallelAction(scoreThirdSpecimen,trafficLight.disable()),
                    raiseArmForSpecimen(),
                    scoreSpecimen(rear_distance));
        }else if(SPECIMEN_COUNT == 2){
            auto = new SequentialAction(scoreFirstSpecimen,
                    raiseArmForSpecimen(),
                    scoreSpecimen(rear_distance),
                    throwFirstSample,
                    drive.moveUsingDistance(intake.distance, 2, RobotConstants.TOO_CLOSE, RobotConstants.TOO_FAR, RobotConstants.GIVE_UP),
                    intake.yeetSample(),
                    throwSecondSample,
                    drive.moveUsingDistance(intake.distance, 2, RobotConstants.TOO_CLOSE, RobotConstants.TOO_FAR, RobotConstants.GIVE_UP),
                    intake.yeetSample(),
                    clearThirdSample,
                    trafficLight.warnHuman(),
                    drive.moveUsingDistance(intake.distance, 2, RobotConstants.TOO_CLOSE, RobotConstants.TOO_FAR, 12),
                    new ParallelAction(scoreSecondSpecimen,trafficLight.disable()),
                    raiseArmForSpecimen(),
                    scoreSpecimen(rear_distance));
        }else{
            auto = new SequentialAction(
                    scoreFirstSpecimen,
                    raiseArmForSpecimen(),
                    scoreSpecimen(rear_distance)
            );
        }

        intake.moveArm(0);
        while(intake.arm.isBusy()){
            intake.update();
        }
        telemetry.clear();
        telemetry.addLine("Arm Lowered, ready to go");
        telemetry.update();

        waitForStart();


        Actions.runBlocking(
                new ParallelAction(
                        intake.updateAction(),
                        trafficLight.updateAction(),
                        rear_distance.updateAction(),
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