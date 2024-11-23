package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
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
    public void initAuto(){
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        timer = new ElapsedTime();
        trafficLight = new TrafficLight("front", hardwareMap, telemetry, RobotConstants.red_led, RobotConstants.green_led, timer);
        intake = new Intake(hardwareMap, telemetry, timer, trafficLight);
        intake.init();
        intake.slides.setReachedRange(30);
        intake.calculateOffset();
        intake.moveClaw(1);
    }
    public Action raiseArmForSpecimen(){
        return new ParallelAction(
                intake.slideAction(360),
                intake.armAction(1400),
                new InstantAction(() -> intake.moveWrist(0.8))
        );
    }
    public Action eternalAction(){
        return new SleepAction(30);
    }

    public Action scoreSpecimen(Distance rear_distance){
        return new SequentialAction(
                drive.moveUsingDistance(rear_distance, 4, 3.5, 4.5, false),
                intake.slideAction(50),
                drive.moveUsingDistance(rear_distance, 9, 0.15, false),
                intake.grab(0.73),
                new InstantAction(() -> intake.moveWrist(RobotConstants.floor_pickup_position + 0.1))
        );
    }
    public Action collectSampleAndScore(Action sampleScore, double ending_claw_pos){
        return new SequentialAction(
                drive.moveUsingDistance(intake.distance, RobotConstants.TARGET, RobotConstants.TOO_CLOSE, RobotConstants.TOO_FAR, RobotConstants.GIVE_UP),
                intake.grab(1),
                new ParallelAction(
                        intake.raiseArm(),
                        sampleScore
                ),
                intake.score(),
                new InstantAction(() -> intake.moveClaw(ending_claw_pos))
        );
    }
}
