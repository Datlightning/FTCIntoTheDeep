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
        Pose2d beginPose = new Pose2d(10, -64, Math.toRadians(270 ));
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

        FailoverAction pickupAfterDistance1 = new FailoverAction(pickupAfterDistance(RobotConstants.TOO_FAR, intake.distance), new InstantAction(() -> intake.distance.setOn(false)), false);

        FailoverAction pickupAfterDistance2 = new FailoverAction(pickupAfterDistance(RobotConstants.TOO_FAR, intake.distance), new InstantAction(() -> intake.distance.setOn(false)), false);

        FailoverAction pickupAfterDistance3 = new FailoverAction(intake.distance.waitAction(4),new InstantAction(() -> intake.distance.setOn(false)), false );

        FailoverAction stopBeforeDistance1 = new FailoverAction(rear_distance.waitAction(5),new InstantAction(() -> rear_distance.setOn(false)), false);

        FailoverAction stopBeforeDistance2 = new FailoverAction(rear_distance.waitAction(5),new InstantAction(() -> rear_distance.setOn(false)), false);

        FailoverAction stopBeforeDistance3 = new FailoverAction(rear_distance.waitAction(5),new InstantAction(() -> rear_distance.setOn(false)), false );



        double CHAMBER_Y = -35.5;
        TrajectoryActionBuilder scoreFirstSpecimenPath = drive.actionBuilder(beginPose)
                .afterTime(1, new InstantAction(() -> intake.moveWrist(RobotConstants.specimen_deliver)))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(0,CHAMBER_Y), Math.toRadians(90), null, smartScore);
        telemetry.addLine("score first specimen generated");
        telemetry.update();
        TrajectoryActionBuilder clearSamples = scoreFirstSpecimenPath.endTrajectory().fresh()
//                .setReversed(false)
                .setReversed(true)
                .setTangent(Math.toRadians(270))
                .splineTo(new Vector2d(0, -42), Math.toRadians(270))

                .splineToSplineHeading(new Pose2d(10, -43,0), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(35, -31), Math.toRadians(90), highModeVel, highMode)
                .splineToConstantHeading(new Vector2d(43, -13), Math.toRadians(0), highModeVel, highMode)
                .splineToConstantHeading(new Vector2d(48, -17), Math.toRadians(270), highModeVel, highMode)

                .lineToYConstantHeading(-58, new TranslationalVelConstraint(120))
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(44, -18), Math.toRadians(0), highModeVel, highMode)
                .splineToConstantHeading(new Vector2d(55, -17), Math.toRadians(270), new TranslationalVelConstraint(10), new ProfileAccelConstraint(-14,20))
                .lineToYConstantHeading(-58, new TranslationalVelConstraint(120))
                .setTangent(Math.toRadians(90))

//                .splineToLinearHeading(new Pose2d(32, -45, Math.toRadians(90)), Math.toRadians(270), new TranslationalVelConstraint(120))
                .splineToLinearHeading(new Pose2d(32, -64, Math.toRadians(0)), Math.toRadians(270), new TranslationalVelConstraint(120), new ProfileAccelConstraint(-14,40));

        TrajectoryActionBuilder backToStartPath = clearSamples.endTrajectory().fresh()
                        .setTangent(Math.toRadians(90))
                        .lineToY(-44)
                        .splineToLinearHeading(beginPose, Math.toRadians(270), new TranslationalVelConstraint(20), new ProfileAccelConstraint(-14,20));
        telemetry.addLine("clear samples generates");
        telemetry.update();
        TrajectoryActionBuilder moveForward1 = clearSamples.endTrajectory().fresh()
                .setTangent(0)
                .stopAndAdd(new InstantAction(pickupAfterDistance1::enable))
                .lineToX(42, new TranslationalVelConstraint(30), new ProfileAccelConstraint(-7,30))
                .stopAndAdd(new InstantAction(pickupAfterDistance1::failover));

        TrajectoryActionBuilder scoreSecondSpecimenPath = clearSamples.endTrajectory().fresh()
                .setReversed(true)
                .afterTime(0.5, new InstantAction(() -> intake.moveWrist(RobotConstants.specimen_deliver)))
                .afterTime(2, new InstantAction(stopBeforeDistance1::enable))
                .splineToLinearHeading(new Pose2d(2, CHAMBER_Y, Math.toRadians(270)), Math.toRadians(90), null,smartScore)
                .stopAndAdd(new InstantAction(stopBeforeDistance1::failover));

        TrajectoryActionBuilder pickThirdSpecimenPath = scoreSecondSpecimenPath.endTrajectory().fresh()
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(2, -42, Math.toRadians(270)), Math.toRadians(270))
                .afterTime(0.1, new InstantAction(pickupAfterDistance2::enable))
                .splineToSplineHeading(new Pose2d(25, -64, Math.toRadians(0)), Math.toRadians(0), new TranslationalVelConstraint(50), new ProfileAccelConstraint(-15,40));



        TrajectoryActionBuilder moveForward2 = pickThirdSpecimenPath.endTrajectory().fresh()
                .lineToX(42, new TranslationalVelConstraint(30), new ProfileAccelConstraint(-7,30))
                .stopAndAdd(new InstantAction(pickupAfterDistance2::failover));

        TrajectoryActionBuilder scoreThirdSpecimenPath = pickThirdSpecimenPath.endTrajectory().fresh()
                .setReversed(true)
                .afterTime(0.5, new InstantAction(() -> intake.moveWrist(RobotConstants.specimen_deliver)))
                .afterTime(2, new InstantAction(stopBeforeDistance2::enable))
                .splineToLinearHeading(new Pose2d(7, CHAMBER_Y, Math.toRadians(270)), Math.toRadians(90), null, smartScore)
                .stopAndAdd(new InstantAction(stopBeforeDistance2::failover));

        TrajectoryActionBuilder pickFourthSpecimenPath = scoreThirdSpecimenPath.endTrajectory().fresh()
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(7, -42, Math.toRadians(270)), Math.toRadians(270))
                .afterTime(0.1, new InstantAction(pickupAfterDistance3::enable))
                .splineToSplineHeading(new Pose2d(25, -64, Math.toRadians(0)), Math.toRadians(0), new TranslationalVelConstraint(50), new ProfileAccelConstraint(-15,40));

        TrajectoryActionBuilder moveForward3 = pickFourthSpecimenPath.endTrajectory().fresh()
                .lineToX(42, new TranslationalVelConstraint(30), new ProfileAccelConstraint(-7,30))
                .stopAndAdd(new InstantAction(pickupAfterDistance3::failover));

        TrajectoryActionBuilder scoreFourthSpecimenPath = pickFourthSpecimenPath.endTrajectory().fresh()
                .setReversed(true)
                .afterTime(0.5, new InstantAction(() -> intake.moveWrist(RobotConstants.specimen_deliver)))
                .afterTime(2, new InstantAction(stopBeforeDistance3::enable))
                .splineToLinearHeading(new Pose2d(10, CHAMBER_Y, Math.toRadians(270)), Math.toRadians(90))
                .stopAndAdd(new InstantAction(stopBeforeDistance3::failover));
        TrajectoryActionBuilder moveBackPath =scoreFourthSpecimenPath.endTrajectory().fresh()
                .splineToConstantHeading(new Vector2d(10,-40), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(40,-40), Math.toRadians(0));
        telemetry.addLine("line 125");
        telemetry.update();
        Action clearSampleAction = clearSamples.build();
        telemetry.addLine("line shhh");
        FailoverAction scoreFirstSpecimen = new FailoverAction(scoreFirstSpecimenPath.build(), new InstantAction(() -> drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0))));
        FailoverAction scoreSecondSpecimen = new FailoverAction(scoreSecondSpecimenPath.build(), new InstantAction(() -> drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0))));
        telemetry.update();
        Action pickThirdSpecimen = pickThirdSpecimenPath.build();
        FailoverAction scoreThirdSpecimen = new FailoverAction(scoreThirdSpecimenPath.build(), new InstantAction(() -> drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0))));
        Action pickFourthSpecimen = pickFourthSpecimenPath.build();
        FailoverAction scoreFourthSpecimen =  new FailoverAction(scoreFourthSpecimenPath.build(), new InstantAction(() -> drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0))));
        FailoverAction moveForwardAction1= new FailoverAction(moveForward1.build(), new InstantAction(() -> drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0))));
        FailoverAction moveForwardAction2 = new FailoverAction(moveForward2.build(), new InstantAction(() -> drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0))));
        FailoverAction moveForwardAction3 = new FailoverAction(moveForward3.build(), new InstantAction(() -> drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0))));
        Action moveBack = moveBackPath.build();
        Action backToStart = backToStartPath.build();

        Action auto;
        SPECIMEN_COUNT = 4;
//        if(SPECIMEN_COUNT == 4){
            auto = new SequentialAction(
                    new ParallelAction(
                            new SequentialAction(
                                    intake.armAction(ARM_LIMIT , 100),
                                    new ParallelAction(
                                            intake.armAction(ARM_LIMIT ),
                                            intake.slideAction(100)
                                    )
                            ),
                            afterDistance(5,rear_distance, new InstantAction(scoreFirstSpecimen::failover)),
                            scoreFirstSpecimen
                    ),
                    intake.slideAction(0),
                    new ParallelAction(
                        clearSampleAction,
                        new SequentialAction(
                                openClawAfterDistance(7.5, rear_distance),
                                new InstantAction(() -> intake.moveWrist(0)),
                                intake.armAction(0)

                        )
                    ),


                    new InstantAction(() -> intake.moveWrist(RobotConstants.floor_pickup_position)),
                    trafficLight.warnHuman(),
                    new ParallelAction(
                    moveForwardAction1,
                        new SequentialAction(
                                            pickupAfterDistance1,
                                            new InstantAction(moveForwardAction1::failover)
                        )
                    ),
//                    drive.moveUsingDistance(intake.distance, RobotConstants.TARGET, RobotConstants.TOO_CLOSE, RobotConstants.TOO_FAR, 12),
                    intake.grab(RobotConstants.claw_closed),
                    new ParallelAction(
                            new ParallelAction(
                                    scoreSecondSpecimen,
                                    new SequentialAction(
                                            stopBeforeDistance1,
                                            new InstantAction(scoreSecondSpecimen::failover)
                                    )
                            ),
                            trafficLight.disable(),
                            raiseArmForSpecimen()
                    ),

                    new InstantAction(() -> rear_distance.setOn(true)),
                    new ParallelAction(
                        intake.slideAction(0)
                            ),
                    new InstantAction(() -> intake.slides.setExitWithTime(false)),
                    new ParallelAction(
                            pickThirdSpecimen,
                            new SequentialAction(
                                    openClawAfterDistance(8.5, rear_distance),
                                    new InstantAction(() -> intake.moveWrist(RobotConstants.floor_pickup_position)),
                                    intake.armAction(0)
                            )
                    ),
                new InstantAction(() -> intake.arm.setExitWithTime(false)),
                trafficLight.warnHuman(),
                    new ParallelAction(
                        moveForwardAction2,
                            new SequentialAction(
                                    pickupAfterDistance2,
                                    new InstantAction(moveForwardAction2::failover)
                            )
                    ),
//                    drive.moveUsingDistance(intake.distance, RobotConstants.TARGET, RobotConstants.TOO_CLOSE, RobotConstants.TOO_FAR, 12),
                    intake.grab(RobotConstants.claw_closed),
                    new ParallelAction(
                            new ParallelAction(
                                    scoreThirdSpecimen,
                                    new SequentialAction(
                                            stopBeforeDistance2,
                                            new InstantAction(scoreThirdSpecimen::failover)
                                    )
                            ),
                            trafficLight.disable(),
                            raiseArmForSpecimen()),
                    new InstantAction(() -> rear_distance.setOn(true)),
                    new ParallelAction(
                            intake.slideAction(0)
                    ),
                    new InstantAction(() -> intake.slides.setExitWithTime(false)),
                    new ParallelAction(
                            pickFourthSpecimen,
                            new SequentialAction(
                                    openClawAfterDistance(8.5, rear_distance),
                                    new InstantAction(() -> intake.moveWrist(RobotConstants.floor_pickup_position)),
                                    intake.armAction(0)
                            )
                    ),
                    new InstantAction(() -> intake.arm.setExitWithTime(false)),
                    trafficLight.warnHuman(),
                    new ParallelAction(
                            moveForwardAction3,
                            new SequentialAction(
                                    pickupAfterDistance3,
                                    new InstantAction(moveForwardAction3::failover)
                            )
                    ),
//                    drive.moveUsingDistance(intake.distance, RobotConstants.TARGET, RobotConstants.TOO_CLOSE, RobotConstants.TOO_FAR, 12),
                    intake.grab(RobotConstants.claw_closed),

                    new ParallelAction(
                            new ParallelAction(
                                    scoreFourthSpecimen,
                                    new SequentialAction(
                                            stopBeforeDistance3,
                                            new InstantAction(scoreFourthSpecimen::failover)
                                    )
                            ),
                            trafficLight.disable(),
                            raiseArmForSpecimen()
                    )
//                    new InstantAction(() -> intake.arm.setExitWithTime(false)),
//                    trafficLight.warnHuman()
////                    drive.moveUsingDistance(intake.distance, RobotConstants.TARGET, RobotConstants.TOO_CLOSE, RobotConstants.TOO_FAR, 12),
//                    moveForwardAction3,
//                    new ParallelAction(scoreFourthSpecimen,trafficLight.disable(),raiseArmForSpecimen()),
//                    new ParallelAction(
//                            intake.slideAction(0),
//                            intake.armAction(ARM_LIMIT)
//                    ),
//                    new InstantAction(() -> rear_distance.setOn(true)),
//                    new ParallelAction(
//                            moveBack,
//                            new SequentialAction(
//                                    openClawAfterDistance(7.5, rear_distance),
//                                    new InstantAction(() -> intake.moveWrist(RobotConstants.floor_pickup_position)),
//                                    intake.armAction(0)
//
//                            )
//                    )

                    );
//        }
//        else if(SPECIMEN_COUNT == 3){
//            auto = new SequentialAction(scoreFirstSpecimen,
//                    raiseArmForSpecimen(),
//                    scoreSpecimen(rear_distance),
//                    throwFirstSample,
//                    drive.moveUsingDistance(intake.distance, 2, RobotConstants.TOO_CLOSE, RobotConstants.TOO_FAR, RobotConstants.GIVE_UP),
//                    intake.yeetSample(),
//                    throwSecondSample,
//                    drive.moveUsingDistance(intake.distance, 2, RobotConstants.TOO_CLOSE, RobotConstants.TOO_FAR, RobotConstants.GIVE_UP),
//                    intake.yeetSample(),
//                    clearThirdSample,
//                    trafficLight.warnHuman(),
//
//                    drive.moveUsingDistance(intake.distance, 2, RobotConstants.TOO_CLOSE, RobotConstants.TOO_FAR, 12),
//                    new ParallelAction(scoreSecondSpecimen,trafficLight.disable()),
//                    raiseArmForSpecimen(),
//                    scoreSpecimen(rear_distance),
//                    pickThirdSpecimen,
//                    trafficLight.warnHuman(),
//
//                    drive.moveUsingDistance(intake.distance, 2, RobotConstants.TOO_CLOSE, RobotConstants.TOO_FAR, 12),
//                    new ParallelAction(scoreThirdSpecimen,trafficLight.disable()),
//                    raiseArmForSpecimen(),
//                    scoreSpecimen(rear_distance));
//        }
//        else if(SPECIMEN_COUNT == 2){
//            auto = new SequentialAction(scoreFirstSpecimen,
//                    raiseArmForSpecimen(),
//                    scoreSpecimen(rear_distance),
//                    throwFirstSample,
//                    drive.moveUsingDistance(intake.distance, 2, RobotConstants.TOO_CLOSE, RobotConstants.TOO_FAR, RobotConstants.GIVE_UP),
//                    intake.yeetSample(),
//                    throwSecondSample,
//                    drive.moveUsingDistance(intake.distance, 2, RobotConstants.TOO_CLOSE, RobotConstants.TOO_FAR, RobotConstants.GIVE_UP),
//                    intake.yeetSample(),
//                    clearThirdSample,
//                    trafficLight.warnHuman(),
//                    drive.moveUsingDistance(intake.distance, 2, RobotConstants.TOO_CLOSE, RobotConstants.TOO_FAR, 12),
//                    new ParallelAction(scoreSecondSpecimen,trafficLight.disable()),
//                    raiseArmForSpecimen(),
//                    scoreSpecimen(rear_distance));
//        }
//        else{
//            auto = new SequentialAction(
//                    scoreFirstSpecimen,
//                    raiseArmForSpecimen(),
//                    scoreSpecimen(rear_distance)
//            );
//        }


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