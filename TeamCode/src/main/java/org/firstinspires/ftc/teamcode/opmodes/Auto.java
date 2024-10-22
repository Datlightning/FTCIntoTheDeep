package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
@Config
public class Auto extends LinearOpMode {
    enum START{
        RED_SPECIMEN,
        RED_SAMPLE,
        BLUE_SAMPLE,
        BLUE_SPECIMEN
    }
    START position;
    ElapsedTime timer;
    Intake intake;
    MecanumDrive mecanumDrive;
    Gamepad currentGamepad1,  previousGamepad1;
    TrajectorySequence scoreFirst, pickupSecond, scoreSecond, pickupThird, scoreThird, pickupFourth, scoreFourth;
    private boolean exit = false;
    enum STATES{
        MOVE_FIRST,
        EXTEND_SLIDES,
        DELIVER,
        RETRACT_SLIDES,
        MOVE_SECOND,
        PICK_UP,

    }
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        timer = new ElapsedTime();
        intake = new Intake(hardwareMap, telemetry, timer);
        mecanumDrive = new MecanumDrive(hardwareMap);

        currentGamepad1 = new Gamepad();
        previousGamepad1 = new Gamepad();
        telemetry.clear();
        telemetry.addLine("x for Blue Sample");
        telemetry.addLine("a for Blue Specimen");
        telemetry.addLine("b for Red Sample");
        telemetry.addLine("y for Red Specimen");
        telemetry.update();
        while (!isStopRequested() && !isStarted()) {
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);
            while (!isStopRequested() && !isStarted()) {
                sleep(20);
                if (currentGamepad1.x && !previousGamepad1.x) {
                    telemetry.clear();
                    telemetry.addLine("Selected Blue Sample");

                    position = START.BLUE_SAMPLE;
                    break;
                }
                if (currentGamepad1.y && !previousGamepad1.y) {
                    telemetry.clear();
                    telemetry.addLine("Selected Red Specimen");
                    telemetry.update();

                    position = START.RED_SPECIMEN;

                    break;
                }
                if (currentGamepad1.a && !previousGamepad1.a) {
                    telemetry.clear();
                    telemetry.addLine("Selected Blue Specimen");

                    position = START.BLUE_SPECIMEN;


                    break;
                }
                if (currentGamepad1.b && !previousGamepad1.b) {
                    telemetry.clear();
                    telemetry.addLine("Selected Red Sample");
                    position = START.RED_SAMPLE;

                    break;
                }
                telemetry.update();
            }
            telemetry.addLine("Press x to confirm, press y to restart");
            telemetry.update();
            while (!isStopRequested()) {
                if (currentGamepad1.x && !previousGamepad1.x) {
                    exit = true;
                    break;
                }
                if (currentGamepad1.y && !previousGamepad1.y) {
                    exit = false;
                    break;
                }
                sleep(20);
            }
            if (exit) {
                break;
            }
        }
        switch(position){
            case RED_SAMPLE:
                scoreFirst = mecanumDrive.trajectorySequenceBuilder(new Pose2d(-38, -63, 0))
                    .setReversed(true)
                    .splineToConstantHeading(new Vector2d(-45,-50), Math.toRadians(180))
                    .splineTo(new Vector2d(-53,-53), Math.toRadians(225))
                    .setReversed(false)
                    .build();
                pickupSecond = mecanumDrive.trajectorySequenceBuilder(scoreFirst.end())
                        .splineTo(new Vector2d(-48,-36), Math.toRadians(90))
                        .build();
                scoreSecond = mecanumDrive.trajectorySequenceBuilder(pickupSecond.end())
                        .setReversed(true)
                        .splineTo(new Vector2d(-53,-53), Math.toRadians(225))
                        .build();
                pickupThird = mecanumDrive.trajectorySequenceBuilder(scoreSecond.end())
                        .setReversed(false)
                        .splineTo(new Vector2d(-58,-36), Math.toRadians(90))
                        .build();
                scoreThird = mecanumDrive.trajectorySequenceBuilder(pickupThird.end())
                        .setReversed(true)
                        .splineTo(new Vector2d(-53,-53), Math.toRadians(225))
                        .build();
                pickupFourth = mecanumDrive.trajectorySequenceBuilder(scoreThird.end())
                        .forward(0.1)
                        .build();
                scoreFourth = mecanumDrive.trajectorySequenceBuilder(pickupFourth.end())
                        .setReversed(true)
                        .splineTo(new Vector2d(-53,-53), Math.toRadians(225))
                        .build();


        }
        waitForStart();


    }
}
