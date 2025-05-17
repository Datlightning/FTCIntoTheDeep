package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class SkystoneScrimmage extends LinearOpMode { // SSR = Sky Stone Robot // ALSO HAS THE CLAW CODE BUILT IN

    // Drive Train Motor //

    public DcMotor RightFrontMotor;
    public DcMotor LeftFrontMotor;
    public DcMotor RightBackMotor;
    public DcMotor LeftBackMotor;

    // Claw Motor //

    public Servo ClawServo; // The "pincher" part of Samanyu's claw
    public CRServo RPServo; // The motor that moves the rack and pinion of Samanyu's claw

    @Override
    public void runOpMode() {// In the Driver Hub this configuration will be called: File_2_DriverControlsAndClaw_SSR <-- or smth. like that

        // Drive Train Motors //

        RightFrontMotor = hardwareMap.get(DcMotor.class, "RFM");
        LeftFrontMotor = hardwareMap.get(DcMotor.class, "LFM");
        RightBackMotor = hardwareMap.get(DcMotor.class, "RBM");
        LeftBackMotor = hardwareMap.get(DcMotor.class, "LBM");

        // Claw Motors //

        ClawServo = hardwareMap.get(Servo.class, "CLS");
        RPServo = hardwareMap.get(CRServo.class, "RPS");

        waitForStart();

        // Claw Variables //
        // Please do not change the toggle unless you know what you are doing

        boolean isNormal = false; // Toggle driving settings // click 'y' to toggle // starts at Araya's setting
        boolean yPressedLast = false; // Debounce key; basically keeps track of when the 'y' key is pressed

        while (!isStopRequested()) {
            if (gamepad1.y && !yPressedLast) { // If you want to change the hotkey from 'y' to something else, just change it here...
                isNormal = !isNormal;
                yPressedLast = true;
            } else if (!gamepad1.y) { // and here...
                yPressedLast = false;
            }

            if (isNormal) {

                // Gautham's preferred way to drive //

                /*
                 * How to use:
                 * - Apply throttle to the joystick to your left [UP/DOWN ONLY].
                 * - Pushing this forward makes the robot go forward.
                 * - Pushing this backward makes the robot go backward.
                 * - Apply throttle to the joystick to your right [LEFT/RIGHT ONLY].
                 * - Pushing this left makes the robot turn left.
                 * - Pushing this right makes the robot turn right.
                 */

                RightFrontMotor.setPower(-gamepad1.left_stick_y);
                LeftFrontMotor.setPower(gamepad1.left_stick_y);
                RightBackMotor.setPower(-gamepad1.left_stick_y);
                LeftBackMotor.setPower(gamepad1.left_stick_y);

                RightFrontMotor.setPower(-gamepad1.right_stick_x * 2.0);
                LeftFrontMotor.setPower(-gamepad1.right_stick_x * 2.0);
                RightBackMotor.setPower(-gamepad1.right_stick_x * 2.0);
                LeftBackMotor.setPower(-gamepad1.right_stick_x * 2.0);
            } else {

                // Araya's preferred way to drive //

                /*
                 * How to use:
                 * - Push both the joysticks forward to make the robot go forward.
                 * - Push both the joysticks backward to make the robot go backward.
                 * - Push one up and one down to rotate on spot.
                 * Limitations:
                 * - Cannot turn and move at the same time.
                 * - Requires operator to consciously give the same amount of throttle to both joysticks.
                 */

                RightFrontMotor.setPower(-gamepad1.right_stick_y);
                RightBackMotor.setPower(-gamepad1.right_stick_y);

                LeftFrontMotor.setPower(gamepad1.left_stick_y);
                LeftBackMotor.setPower(gamepad1.left_stick_y);
            }

            // Claw Implementation //

            if (gamepad2.a) // This is for the Samanyu's claw to open
                ClawServo.setPosition(1); // Extended

            if (gamepad2.b) // This is for the Samanyu's claw to close
                ClawServo.setPosition(0.5); // Closed

            RPServo.setPower(gamepad2.right_stick_y); // This is for the rack and pinion of Samanyu's claw
        }
    }
}

