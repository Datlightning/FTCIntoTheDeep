package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp
public class SlideTesting extends LinearOpMode {
    Servo slide_left;
    Servo slide_right;
    public static double offset = 0;
    public static double right_pos = .5; //Min is 0.32
    public static double left_pos = .5; // Max is 0.68



    @Override
    public void runOpMode() throws InterruptedException {
        slide_left = hardwareMap.get(Servo.class, "slide_left");
        slide_right = hardwareMap.get(Servo.class, "slide_right");

        boolean prevA = false;
        boolean prevB = false;
        boolean prevX = false;
        boolean prevY = false;

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            slide_left.setPosition(left_pos - offset);
            slide_right.setPosition(right_pos + offset);

            if (gamepad1.left_trigger >= 0.5) {
                left_pos = 0.57;
                right_pos = .43;
            }

            if(gamepad1.right_trigger >= 0.5) {
                left_pos = 0.19;
                right_pos = 0.81;
            }

            if (gamepad1.a && !prevA) {
                left_pos += 0.1;
                right_pos -= 0.1;
            }
            prevA = gamepad1.a;

            // Button B: Increment by 0.01
            if (gamepad1.b && !prevB) {
                left_pos += 0.01;
                right_pos -= 0.01;
            }
            prevB = gamepad1.b;

            // Button X: Decrement by 0.1
            if (gamepad1.x && !prevX) {
                left_pos -= 0.1;
                right_pos += 0.1;
            }
            prevX = gamepad1.x;

            // Button Y: Decrement by 0.01
            if (gamepad1.y && !prevY) {
                left_pos -= 0.01;
                right_pos += 0.01;
            }
            prevY = gamepad1.y;

            // Cap the positions between 0 and 1
            left_pos = Math.max(.19, Math.min(.57, left_pos));
            right_pos = Math.max(.43, Math.min(.81, right_pos));

            // Telemetry
            telemetry.addData("Left Pos", left_pos);
            telemetry.addData("Right Pos", right_pos);
            telemetry.update();
        }
    }
}
