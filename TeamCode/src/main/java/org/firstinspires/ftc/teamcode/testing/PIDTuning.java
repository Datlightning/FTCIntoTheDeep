package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Intake;

import java.util.concurrent.TimeUnit;

@TeleOp
@Config
public class PIDTuning extends LinearOpMode {
    DcMotor motor;
    double error, lastError;

    public static double power = 0;
    public static int target_position = 0;
    private static int past_target = 0;
    public static double P = 0, I =0, D = 0, F=0;
    public static String motor_name = "arm_motor";
    ElapsedTime timer;
    double out = 0;
    public static double integralSum = 0;
    double max_integral = 0.4;
    double time_passed = 0, time_stop = 0;
    public static double CEILING = 900, FLOOR = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        motor = hardwareMap.get(DcMotor.class, motor_name);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        timer = new ElapsedTime();
        waitForStart();
        while (!isStopRequested() && opModeIsActive()) {
            // Update time passed early on
            time_passed = timer.seconds() - time_stop;
            time_stop = timer.seconds();

// If manual power is applied, skip the PID calculation
            if (power != 0){
                motor.setPower(power);
                continue;
            }

// Reset integral sum if target position has changed
            if(past_target != target_position){
                integralSum = 0;
            }
            past_target = target_position;

// Calculate error first
            error = target_position - motor.getCurrentPosition();

// Proportional term
            out = P * error;

// Integral term with windup protection
            integralSum += error * time_passed;
            if (I * integralSum <= max_integral){
                out += I * integralSum;
            } else {
                out += max_integral;
            }

// Derivative term (with time_passed already calculated)
            if(time_passed > 0) {
                double derivative = (error - lastError) / time_passed;
                out += D * derivative;
            }

// Feedforward term
            out += F;

// Set motor power output
            motor.setPower(out);

// Update last error for the next iteration
            lastError = error;

// Telemetry for debugging
            telemetry.addData("Current Position", motor.getCurrentPosition());
            telemetry.addData("Output Power", out);
            telemetry.addData("Target Position", target_position);
            telemetry.addData("Floor", FLOOR);
            telemetry.addData("Ceil", CEILING);
            telemetry.update();

        }

    }
}
