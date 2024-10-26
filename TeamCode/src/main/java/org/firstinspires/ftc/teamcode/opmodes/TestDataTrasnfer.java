package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

@Config
@Autonomous
public class TestDataTrasnfer extends LinearOpMode {
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
    private double past_slide_current = 0.0001, slide_current = 0.0001;
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
        intake.foldWrist();
        intake.arm.setUseMotionProfile(true);
        intake.slides.setUseMotionProfile(true);
        intake.init();
        sleep(200);
        intake.calculateOffset();
        RobotConstants.auto_transfer = true;
        RobotConstants.intake = intake;
        telemetry.addData("Offset", intake.getOffset());
        telemetry.update();
        RobotConstants.offset = intake.getOffset();
//        intake.closeClaw();
       waitForStart();

    return;


    }
}
