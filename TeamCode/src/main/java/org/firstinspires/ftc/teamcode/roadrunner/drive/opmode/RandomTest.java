package org.firstinspires.ftc.teamcode.roadrunner.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Disabled
@Autonomous(group = "drive")
public class RandomTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        TrajectorySequence traj = drive.trajectorySequenceBuilder(new Pose2d(-38, -63, Math.toRadians(0)))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-45,-50), Math.toRadians(180))
//                    .addTemporalMarker(() -> intake.moveArm(1500))
                .splineTo(new Vector2d(-53,-53), Math.toRadians(225))
                .setReversed(false)
                .splineTo(new Vector2d(-48,-36), Math.toRadians(90))

                .setReversed(true)
                .splineTo(new Vector2d(-53,-53), Math.toRadians(225))
                .setReversed(false)
                .splineTo(new Vector2d(-58,-36), Math.toRadians(90))
                .setReversed(true)
                .splineTo(new Vector2d(-53,-53), Math.toRadians(225))
                .build();
        drive.setPoseEstimate(traj.start());
        drive.followTrajectorySequence(traj);



    }
}
