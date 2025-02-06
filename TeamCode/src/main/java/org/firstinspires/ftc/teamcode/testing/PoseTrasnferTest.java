package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.MecaTank;

@Config
public class PoseTrasnferTest extends TestingOpMode {

    MecanumDrive mecanumDrive;
    MecaTank mecaTank;
    public static Pose2d pose = new Pose2d(0,0, Math.toRadians(270));
    @Override
    public void runOpMode() throws InterruptedException {
        makeTelemetry();
        mecanumDrive = new MecanumDrive(hardwareMap, pose);
        mecaTank = new MecaTank(hardwareMap, telemetry);

        waitForStart();
        while(opModeIsActive() && !isStopRequested()){
            mecaTank.update();
            mecaTank.setDrivePowers(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
            mecaTank.telemetry();
        }

    }
}
