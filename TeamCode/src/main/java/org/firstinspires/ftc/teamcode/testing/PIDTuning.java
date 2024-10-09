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
    public static double P = 0, I =0, D = 0;
    public static String motor_name = "arm_motor";
    ElapsedTime timer;
    double out = 0;
    public static double integralSum = 0;
    double max_integral = 1;
    double time_passed = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        motor = hardwareMap.get(DcMotor.class, motor_name);
        timer = new ElapsedTime();
        waitForStart();
        while (!isStopRequested() && opModeIsActive()) {
            if(power != 0){
                motor.setPower(power);
                continue;
            }
            error = target_position - motor.getCurrentPosition();

            // rate of change of the error
            time_passed = timer.seconds() - time_passed;
            // sum of all error over time
            out = (P * error);
            integralSum = integralSum + (error * time_passed);
            if (integralSum <= max_integral){
                out += (I * integralSum) ;
            }else{
                out += (I * max_integral);
            }



            if(time_passed != 0) {
                double derivative = (error - lastError) / time_passed;
                out += D * derivative;
            }



            motor.setPower(out);
            lastError = error;
            telemetry.update();
        }

    }
}
