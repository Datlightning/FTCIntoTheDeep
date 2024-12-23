package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.RobotConstants.ARM_LIMIT;
import static org.firstinspires.ftc.teamcode.RobotConstants.distance;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Distance;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.TrafficLight;

public abstract class NGAutoOpMode extends LinearOpMode {
    public static ElapsedTime timer;
    public static Intake intake;
    public static MecanumDrive drive;
    public static TrafficLight trafficLight;
    public void initAuto(Pose2d beginPose){
        timer = new ElapsedTime();
        trafficLight = new TrafficLight("front", hardwareMap, telemetry, RobotConstants.red_led, RobotConstants.green_led, timer);
        drive = new MecanumDrive(hardwareMap, beginPose);
        drive.mountTrafficLight(trafficLight);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        RobotConstants.auto_transfer = true;
        intake.mountMecanumDrive(drive);
        intake = new Intake(hardwareMap, telemetry, timer, trafficLight);
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
                        new SequentialAction(
                                new SleepAction(0.3),
                            intake.raiseArm()
                        ),
                      sampleScore

                ),
                new SleepAction(0.2),
                intake.score(),
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
    public Action goToSample(FailoverAction sample, FailoverAction distance){
        return new SequentialAction(
                new InstantAction(() -> intake.moveClaw(RobotConstants.claw_floor_pickup)),
                new ParallelAction(
                        new InstantAction(() -> intake.distance.setOn(true)),
                        new SequentialAction(intake.armAction(0, 1000),
                                sample),
                                new SequentialAction(
                                        distance,
                                        new InstantAction(sample::failover)
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

}
