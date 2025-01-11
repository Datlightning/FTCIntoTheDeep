package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;


@Config
public class DiffyIntakeTest extends LinearOpMode {
    Servo leftServo;
    Servo rightServo;
    Servo clawServo;
    double left_pos = 0;
    double right_pos = 0;
    public static double claw_pos = 0;
    public static double theta1 = 0;
    public static double theta2 = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        leftServo = hardwareMap.get(Servo.class, "left_servo");
        rightServo = hardwareMap.get(Servo.class, "right_servo");
        clawServo = hardwareMap.get(Servo.class, "claw_servo");
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
//COmment for funsies
        while(opModeIsActive()) {

            if (boundClear(theta1, theta2)) {
                left_pos = (theta1/270) - (theta2/270);
                right_pos = 1 - (theta1/270) - (theta2/270);
            }

            leftServo.setPosition(left_pos);
            rightServo.setPosition(right_pos);
            clawServo.setPosition(claw_pos);

            telemetry.addData("theta1", theta1);
            telemetry.addData("theta2", theta2);
            telemetry.addData("left", left_pos);
            telemetry.addData("right", right_pos);
            telemetry.addData("claw", claw_pos);

            telemetry.update();
        }

    }

    public boolean boundClear(double theta1, double theta2) {
        double lowerBound = Math.abs(theta1 - 135) - 135;
        double upperBound = -1 * Math.abs(theta1 - 135) + 135;
        return (theta2 >= lowerBound && theta2 <= upperBound);
    }
}


