package org.firstinspires.ftc.teamcode.opmodes;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.NullAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystems.Camera;

import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Config
@Autonomous
public class AutomaticPickup extends NGAutoOpMode {

    Camera camera;
    double[] data = new double[3];
    public static double slide_transfer_constant = 0;
    public static double robot_transfer_constant = 0;

    public static int slide_position = 0;
    public static int arm_position = 600;
    public static int claw_angle = 0;
    public static int wrist_angle = 0;

    public static double CONSTANT = 0;
    public static double STRAFE_CONSTANT = 0.5;



    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(0, 0, Math.toRadians(0));
        initAuto(beginPose);

        camera = new Camera(hardwareMap, telemetry);
        camera.init();
        intake.openClaw();
        waitForStart();



        Action pickup = new SequentialAction(
                new InstantAction(() -> {vihasRigging.camera(); camera.startCamera();}),
                new SleepAction(1),
                camera.waitForYellow(),
                obtainSampleWithCamera(camera, beginPose)
        );
        Actions.runBlocking(
                new ParallelAction(pickup, intake.updateAction()));








    }
}