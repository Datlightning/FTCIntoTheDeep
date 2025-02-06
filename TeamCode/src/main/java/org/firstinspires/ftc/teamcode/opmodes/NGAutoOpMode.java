package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.RobotConstants.ARM_LIMIT;
import static org.firstinspires.ftc.teamcode.RobotConstants.distance;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.NullAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.RobotAutoDriveToAprilTagOmni;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Distance;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Rigging;
import org.firstinspires.ftc.teamcode.subsystems.TrafficLight;

public abstract class NGAutoOpMode extends LinearOpMode {
    public static ElapsedTime timer;
    public static Intake intake;
    public static MecanumDrive drive;
    public static TrafficLight trafficLight;
//    public static Rigging rigging;
    public void initAuto(Pose2d beginPose){
        timer = new ElapsedTime();
        trafficLight = new TrafficLight("front", hardwareMap, telemetry, RobotConstants.red_led, RobotConstants.green_led, timer);
        drive = new MecanumDrive(hardwareMap, beginPose);
        drive.mountTrafficLight(trafficLight);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        RobotConstants.auto_transfer = true;
        intake = new Intake(hardwareMap, telemetry, timer, trafficLight);
//        rigging = new Rigging(hardwareMap, telemetry, timer);
//        rigging.init();
//        rigging.reset();

        intake.init();

        intake.slides.setReachedRange(30);
//        intake.calculateOffset();
        intake.moveClaw(RobotConstants.claw_closed);
    }
    public Action raiseArmForSpecimen(){
        return new SequentialAction(
                new InstantAction(() -> intake.arm.setExitWithTime(true)),
                new InstantAction(() -> intake.slides.setExitWithTime(true)),
                intake.armAction(ARM_LIMIT , 100),
                new ParallelAction(
                        intake.armAction(ARM_LIMIT ),
                        intake.slideAction(100)
                )
        );
    }
    public Action eternalAction(){
        return new SleepAction(30);

    }
    public class WaitUntilAction implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return (!gamepad1.a);
        }
    }
    public Action betterEternalAction(){

        return new WaitUntilAction();

    }
    public Action scoreSpecimen(Distance rear_distance){
        return new SequentialAction(
                drive.moveUsingDistance(rear_distance, 4.5, 4, 4.8, false),
                intake.slideAction(0)
        );
    }
    public Action newScoreSpecimen(Action action, Distance rear_distance){
        return new SequentialAction(
                intake.slideAction(0),
                new ParallelAction(
                        openClawAfterDistance(7.5, rear_distance),
                        action

                )
        );
    }
    public Action collectSampleAndScore(Action sampleScore, double ending_claw_pos){
        return new SequentialAction(
                drive.moveUsingDistance(intake.distance, RobotConstants.TARGET, RobotConstants.TOO_CLOSE, RobotConstants.TOO_FAR, RobotConstants.GIVE_UP),
                intake.grab(RobotConstants.claw_closed),
                new ParallelAction(
                        intake.raiseArm(),
                        sampleScore
                ),
                intake.score(),
                new InstantAction(() -> intake.moveClaw(ending_claw_pos))
        );
    }
    public Action collectSampleAndScore(Action sampleScore, double ending_claw_pos, boolean use_distance_move){
        return new SequentialAction(
                intake.grab(RobotConstants.claw_closed),
                new ParallelAction(
                        intake.raiseArm(),
                        sampleScore
                ),
                new SleepAction(0.2),
                intake.score(),
                new InstantAction(() -> intake.moveClaw(ending_claw_pos))
        );
    }


    public Action slideCollectSampleAndScore(Action sampleScore, double ending_claw_pos){
        Intake.moveArmAction armDown = intake.armAction(500, 260);
        return new SequentialAction(
                intake.grab(RobotConstants.claw_closed),
                new ParallelAction(
                        new SequentialAction(
                                    new ParallelAction(
                                        intake.slideAction(200, 400),
                                        new InstantAction(() -> intake.arm.setExitWithTime(true)),
                                        armDown,
                                        new InstantAction(() -> intake.arm.setExitWithTime(false)),
                                        new InstantAction(() -> intake.moveWrist(90))
                                    ),
                                    intake.raiseArm()

                        ),
                        new SequentialAction(
                                sampleScore,
                                new InstantAction(armDown::cancel)
                        )
                ),
                new SleepAction(0.2),
                intake.scoreSlidePickup(),
                new InstantAction(() -> intake.moveClaw(ending_claw_pos))
        );
    }

    public Action goToSample(Action sample){
        return new SequentialAction(
                new InstantAction(() -> intake.moveClaw(RobotConstants.claw_floor_pickup)),
                new ParallelAction(
                        new InstantAction(() -> intake.distance.setOn(true)),
                        new SequentialAction(intake.armAction(0, 1000), sample),
                        intake.slideAction(0)

                )

        );
    }
    public Action goToSampleWithSlides(Action sample){
        Intake.moveArmAction armDown = intake.armAction(300, 700);
        Intake.moveArmAction armDown2 = intake.armAction(200);
        FailoverAction sleep = new FailoverAction(new SleepAction(1.25), new NullAction());
        return new SequentialAction(
                new InstantAction(() ->
                {
                    intake.moveClaw(RobotConstants.claw_flat);
                    intake.moveWrist(180);
                }),

                new ParallelAction(
                        new SequentialAction(

                                armDown,
                                intake.slideAction(1000)
                        ),
                        new SequentialAction(
                                sample,
                                new InstantAction(() -> {intake.useFastPID(true);}),
                                new ParallelAction(
                                        new SequentialAction(armDown2, new InstantAction(sleep::failover)),
                                        new SequentialAction(sleep, new InstantAction(armDown2::cancel))
                                ),
                                new InstantAction(() -> {intake.useFastPID(false);})
                        )

                )
        );
    }
    public Action goToSampleWithSlides(Action sample, FailoverAction delay_action){
        Intake.moveArmAction armDown = intake.armAction(300, 700);
        Intake.moveArmAction armDown2 = intake.armAction(200);
        FailoverAction sleep = new FailoverAction(new SleepAction(1.25), new NullAction());
        return new SequentialAction(
                new InstantAction(() ->
                {
                    intake.moveClaw(RobotConstants.claw_flat);
                    intake.moveWrist(180);
                }),

                new ParallelAction(
                        new SequentialAction(
                                delay_action,
                                armDown,
                                intake.slideAction(1000)
                        ),
                        new SequentialAction(
                                sample,
                                new SequentialAction(
                                        delay_action,
                                    new InstantAction(() -> {intake.useFastPID(true);}),
                                    new ParallelAction(
                                            new SequentialAction(armDown2, new InstantAction(sleep::failover)),
                                            new SequentialAction(sleep, new InstantAction(armDown2::cancel))
                                    ),
                                    new InstantAction(() -> {intake.useFastPID(false);})
                                )
                        )

                )
        );
    }
    public Action goToSampleWithSlides(Action sample,int slide_length){
        return new SequentialAction(
                new InstantAction(() ->
                {
                    intake.moveClaw(RobotConstants.claw_flat);
                    intake.moveWrist(180);
                }),

                new ParallelAction(
                        new SequentialAction(
                                intake.armAction(240, 700),
                                intake.slideAction(slide_length)
                        ),
                        sample
                )
        );
    }
    public Action goToSample(FailoverAction sample, FailoverAction distance){
        return new SequentialAction(
                new InstantAction(() -> intake.moveClaw(RobotConstants.claw_floor_pickup)),
                new ParallelAction(
                        new SequentialAction(
                                intake.armAction(0, 1000),
                                new ParallelAction(
                                        new SequentialAction(
                                                distance,
                                                new InstantAction(sample::failover)
                                        ),
                                        sample
                                )
                        ),
                        intake.slideAction(0)

                )

        );
    }


    public Action openClawAfterDistance(double distance, Distance sensor){
        return new SequentialAction(
          sensor.waitAction(distance),
          new InstantAction(() -> intake.moveClaw(RobotConstants.claw_floor_pickup))
        );
    }
    public Action pickupAfterDistance(double distance, Distance sensor){
        return new SequentialAction(
                sensor.waitAction(distance),
                new InstantAction(() -> intake.moveClaw(RobotConstants.claw_closed))
        );
    }
    public Action afterDistance(double distance, Distance sensor, Action action){
        return new SequentialAction(
                sensor.waitAction(distance),
                action
        );
    }
    public Action telemetryLine(String string){
        return new SequentialAction(
                new InstantAction(() -> telemetry.addLine(string)),
                new InstantAction(() ->
                telemetry.update())
        );
    }
    public class FailoverAction implements Action{
        private final Action mainAction;
        private final Action failoverAction;
        private boolean failedOver = false;
        private boolean on = true;

        public FailoverAction(Action mainAction, Action failoverAction) {
            this.mainAction = mainAction;
            this.failoverAction = failoverAction;
        }
        public FailoverAction(Action mainAction, Action failoverAction, boolean on) {
            this.mainAction = mainAction;
            this.on = on;
            this.failoverAction = failoverAction;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!on){
                return true;
            }
            if (failedOver) {
                return failoverAction.run(telemetryPacket);
            }

            return mainAction.run(telemetryPacket);
        }
        public void enable(){
            on = true;
        }
        public void failover() {
            failedOver = true;
        }
    }
    public Action scoreSpecimen(Action toSpecimen, Action after){
        return new SequentialAction(
                new InstantAction(() -> intake.closeClaw(-0.03)),
                new SleepAction(0.1),
                new ParallelAction(
                        toSpecimen,
                        intake.armAction(1500),
                        intake.slideAction(325),
                        new InstantAction(() -> intake.moveWrist(0))
                ),
                intake.slideAction(800),
                new InstantAction(() -> intake.openClaw()),
                new ParallelAction(
                        after,
                        new SequentialAction(
                                new SleepAction(1),
                                intake.slideAction(0)
                        ),
                        intake.armAction(400),
                        new InstantAction(() -> {intake.moveClaw(RobotConstants.claw_flat); intake.moveWrist(90);})
                )
        );
    }
    public Action transferSample(Action toSample, Action after){
        return new SequentialAction(

                new ParallelAction(
                        toSample,
                        new SequentialAction(
                            intake.armAction(300),
                            new ParallelAction(
                                intake.slideAction(200),
                                new InstantAction(() -> {
                                    intake.turnAndRotateClaw(180,0);
                                    intake.moveClaw(RobotConstants.claw_flat);
                                })
                            )
                        )
                ),
                intake.moveArmFast(200,-0.3),
                new InstantAction(() -> intake.closeClaw()),
                new SleepAction(0.3),
                new ParallelAction(
                    intake.armAction(400),
                    after,
                    intake.slideAction(1000)

                ),
                new InstantAction(() -> intake.moveClaw(RobotConstants.claw_flat))



        );
    }
    public Action transferSample(Action toSample, Action after, double claw_angle, int slide_length){
        return new SequentialAction(

                new ParallelAction(
                        toSample,
                        new SequentialAction(
                                intake.armAction(300, 600),
                                new ParallelAction(
                                    intake.slideAction(slide_length),
                                    new InstantAction(() -> {
                                        intake.turnAndRotateClaw(180, claw_angle);
                                        intake.moveClaw(RobotConstants.claw_flat);
                                    })
                                )
                        )
                ),
                intake.moveArmFast(200,-0.3),
                new InstantAction(() -> intake.closeClaw()),
                new SleepAction(0.3),
                new ParallelAction(
                        intake.armAction(400),
                        after,
                        intake.slideAction(slide_length)

                ),
                new InstantAction(() -> intake.moveClaw(RobotConstants.claw_flat))



        );
    }
    public Action transferSample(Action toSample, Action after, boolean last){
        return new SequentialAction(
                new ParallelAction(
                        toSample,
                        intake.armAction(300),
                        intake.slideAction(200),
                        new InstantAction(() -> {
                            intake.moveWrist(180);
                            intake.moveClaw(RobotConstants.claw_flat);
                        })


                ),
                intake.moveArmFast(200,-0.3),
                new InstantAction(() -> intake.closeClaw()),
                new ParallelAction(
                        intake.armAction(500),
                        after,
                        intake.slideAction(0)

                ),
                new InstantAction(() -> intake.moveClaw(RobotConstants.claw_flat))



        );
    }
}
