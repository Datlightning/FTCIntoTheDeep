package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

@Config
@Autonomous
public class RRPathTest extends LinearOpMode {
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
    TrajectorySequence moveFirst, deliverSample, pickupSecond, scoreSecond, pickupThird, scoreThird, pickupFourth, scoreFourth;
    private boolean exit = false;
    enum STATES{
        MOVE_FIRST,
        EXTEND_SLIDES,
        DELIVER,
        OPEN_CLAW,
        RETRACT_SLIDES,
        LOWER_ARM,
        PICK_UP_FIRST,
        CLOSE_CLAW,
        MOVE_SECOND,
        EXTEND_SLIDES2
    }
    private double delay = 0;
//    STATES auto_position = STATES.MOVE_FIRST;
    private int auto_position = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        timer = new ElapsedTime();
        intake = new Intake(hardwareMap, telemetry, timer);
        mecanumDrive = new MecanumDrive(hardwareMap);

        currentGamepad1 = new Gamepad();
        previousGamepad1 = new Gamepad();
        intake.foldWrist();
        intake.closeClaw();

        telemetry.clear();
        telemetry.addLine("x for Blue Sample");
        telemetry.addLine("a for Blue Specimen");
        telemetry.addLine("b for Red Sample");
        telemetry.addLine("y for Red Specimen");
        telemetry.update();
        while (!isStopRequested()) {
            while(!isStopRequested()) {
                previousGamepad1.copy(currentGamepad1);
                currentGamepad1.copy(gamepad1);
                sleep(20);

                // Check for button presses to select a position
                if (currentGamepad1.x && !previousGamepad1.x) {
                    telemetry.clear();
                    telemetry.addLine("Selected Blue Sample");
                    position = START.BLUE_SAMPLE;
                    telemetry.addLine("Press x to confirm, press y to restart");
                    telemetry.update();
                    break;
                } else if (currentGamepad1.y && !previousGamepad1.y) {
                    telemetry.clear();
                    telemetry.addLine("Selected Red Specimen");
                    position = START.RED_SPECIMEN;
                    telemetry.addLine("Press x to confirm, press y to restart");
                    telemetry.update();
                    break;
                } else if (currentGamepad1.a && !previousGamepad1.a) {
                    telemetry.clear();
                    telemetry.addLine("Selected Blue Specimen");
                    position = START.BLUE_SPECIMEN;
                    telemetry.addLine("Press x to confirm, press y to restart");
                    telemetry.update();
                    break;
                } else if (currentGamepad1.b && !previousGamepad1.b) {
                    telemetry.clear();
                    telemetry.addLine("Selected Red Sample");
                    position = START.RED_SAMPLE;
                    telemetry.addLine("Press x to confirm, press y to restart");
                    telemetry.update();

                    break;
                }


            }
            telemetry.update();
            // Wait for confirmation or restart
            boolean exit = false;
            while (!isStopRequested()) {
                previousGamepad1.copy(currentGamepad1);
                currentGamepad1.copy(gamepad1);
                sleep(20);

                if (currentGamepad1.x && !previousGamepad1.x) {
                    exit = true;  // Confirm selection
                    break;
                } else if (currentGamepad1.y && !previousGamepad1.y) {
                    exit = false; // Restart selection process
                    break;
                }
            }

            if (exit) {
                break; // Exit the loop if confirmed
            }

            telemetry.clear();
            telemetry.addLine("Restarting selection...");
            telemetry.update();
        }

        switch(position){
            case RED_SAMPLE:
                moveFirst = mecanumDrive.trajectorySequenceBuilder(new Pose2d(-38, -63, 0))
                    .setReversed(true)
                    .splineToConstantHeading(new Vector2d(-45,-50), Math.toRadians(180))
                    .addTemporalMarker(() -> intake.moveArm(1410))
                    .splineTo(new Vector2d(-53,-53), Math.toRadians(225))
                    .setReversed(false)
                    .build();

                deliverSample = mecanumDrive.trajectorySequenceBuilder(moveFirst.end())
                        .addTemporalMarker(() -> intake.moveWrist(0.8))
                        .waitSeconds(0.3)
                        .addTemporalMarker(() -> intake.openClaw())
                        .addTemporalMarker(() -> intake.moveWrist(0.5))
                        .waitSeconds(0.3)
                        .addTemporalMarker(() -> intake.moveSlides(0))
                        .addTemporalMarker(() -> intake.closeClaw())
                        .build();

                pickupSecond = mecanumDrive.trajectorySequenceBuilder(deliverSample.end())
                        .addTemporalMarker(() -> intake.moveWrist(0.65))
                        .addTemporalMarker(() -> intake.moveClaw(0.7))
                        .splineTo(new Vector2d(-46,-34), Math.toRadians(90))
                        .addTemporalMarker(() -> intake.closeClaw())
                        .waitSeconds(0.3)
                        .addTemporalMarker(() -> intake.moveWrist(0.5))
                        .setReversed(true)
                        .UNSTABLE_addTemporalMarkerOffset(1.2, () -> intake.moveArm(1470))
                        .splineTo(new Vector2d(-53,-53), Math.toRadians(225))
                        .setReversed(false)
                        .build();


                pickupThird = mecanumDrive.trajectorySequenceBuilder(deliverSample.end())
                        .setReversed(false)
                        .addTemporalMarker(() -> intake.moveWrist(0.65))
                        .addTemporalMarker(() -> intake.moveClaw(0.7))
                        .splineTo(new Vector2d(-58,-36), Math.toRadians(90))
                        .addTemporalMarker(() -> intake.closeClaw())
                        .waitSeconds(0.3)
                        .addTemporalMarker(() -> intake.moveWrist(0.5))
                        .setReversed(true)
                        .UNSTABLE_addTemporalMarkerOffset(1.2, () -> intake.moveArm(1470))
                        .splineTo(new Vector2d(-53,-53), Math.toRadians(225))
                        .setReversed(false)
                        .build();

                pickupFourth = mecanumDrive.trajectorySequenceBuilder(deliverSample.end())
                        .forward(0.1)
                        .build();

                mecanumDrive.setPoseEstimate(moveFirst.start());


        }

        waitForStart();
        mecanumDrive.followTrajectorySequenceAsync(moveFirst);

        while(!isStopRequested() && opModeIsActive()){
            switch(auto_position){
                case 0:
                    if(!mecanumDrive.isBusy() && !intake.arm.isBusy()){
                        intake.moveWrist(0.5);
                        auto_position += 1;
                    }
                    break;
                case 1:
                case 5:
                case 9:
                case 13:
                    if(!mecanumDrive.isBusy() && !intake.arm.isBusy()){
                        intake.moveSlides(1400);
                        auto_position += 1;
                    }
                    break;
                case 2:
                case 6:
                case 10:
                case 14:
                    if(!intake.slides.isBusy()){
                        mecanumDrive.followTrajectorySequenceAsync(deliverSample);
                        auto_position += 1;
                    }
                    break;
                case 3:
                case 7:
                case 11:
                case 15:
                    if(!intake.slides.isBusy() && !mecanumDrive.isBusy()){
                        intake.moveArm(0);
                        intake.moveWrist(0.8);
                        auto_position += 1;
                    }
                    break;
                case 4:
                    if(!intake.arm.isBusy()){
                        mecanumDrive.followTrajectorySequenceAsync(pickupSecond);
                        auto_position += 1;
                    }
                    break;
                case 8:
                    if(!intake.arm.isBusy()){
                        mecanumDrive.followTrajectorySequenceAsync(pickupThird);
                        auto_position += 1;
                    }
                    break;
                case 12:
                    if(!intake.arm.isBusy()){
                        mecanumDrive.followTrajectorySequenceAsync(pickupFourth);
                        auto_position += 1;
                    }
                    break;


            }
            mecanumDrive.update();
            intake.update();

        }



    }
}
