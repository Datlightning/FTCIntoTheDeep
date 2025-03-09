package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.Rigging;
import org.firstinspires.ftc.teamcode.subsystems.VihasRigging;

@Config
@TeleOp
public class VihasRiggingTesting extends TestingOpMode {
    VihasRigging vihasRigging;

    public static double servo_position = 0;
    public static boolean servo_on = true;

    Gamepad currentGamepad1, currentGamepad2, previousGamepad1, previousGamepad2;
    @Override
    public void runOpMode() throws InterruptedException {
        makeTelemetry();
        vihasRigging = new VihasRigging(hardwareMap, telemetry);

        currentGamepad1 = new Gamepad();
        currentGamepad2 = new Gamepad();
        previousGamepad1 = new Gamepad();
        previousGamepad2 = new Gamepad();

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            currentGamepad2.copy(gamepad2);
            currentGamepad1.copy(gamepad1);

            if(currentGamepad1.dpad_down && !previousGamepad1.dpad_down){
                servo_position -= 0.05;
            }
            if(currentGamepad1.dpad_up && !previousGamepad1.dpad_up){
                servo_position += 0.05;
            }
            vihasRigging.setServoPosition(servo_position);
            if(currentGamepad1.a && !previousGamepad1.a) {
                if (!vihasRigging.rigging_servo.isPWMEnabled()) {
                    vihasRigging.rigging_servo.enableServo();
                } else {
                    vihasRigging.rigging_servo.disableServo();
                }
            }
            vihasRigging.setManualPower(-gamepad1.left_stick_y);
            telemetry.addData("Servo Position: ", vihasRigging.rigging_servo.getPosition());
            telemetry.addData("Motor Power: ", vihasRigging.rigging_motor.getPower());
            telemetry.update();

        }
    }
}
