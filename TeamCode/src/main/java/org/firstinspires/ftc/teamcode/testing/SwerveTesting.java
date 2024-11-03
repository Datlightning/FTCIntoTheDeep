package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Config
@TeleOp
public class SwerveTesting extends LinearOpMode {

    // Hardware
    private DcMotor driveMotor;      // Driving motor
    private CRServo steeringServo;   // Continuous rotation servo for steering

    // PID coefficients for servo position control
    public static  PIDCoefficients pidCoefficients = new PIDCoefficients(1.0, 0.0, 0.0);  // Adjust as needed
    public static double  targetPosition = 0.5;  // Target position (0 to 1 for servo, assuming it's normalized)
    private double past_target = 0;
    private double last_power = 0;
    public static double power = 0.5;
    @Override
    public void runOpMode() {
        // Initialize hardware
        driveMotor = hardwareMap.get(DcMotor.class, "drive_motor");
        steeringServo = hardwareMap.get(CRServo.class, "servo");

        waitForStart();
        ElapsedTime runtime = new ElapsedTime();

        // Variables for PID control
        double lastError = 0;
        double integral = 0;
        double kp = pidCoefficients.p;
        double ki = pidCoefficients.i;
        double kd = pidCoefficients.d;

        while (opModeIsActive()) {
            kp = pidCoefficients.p;
            ki = pidCoefficients.i;
            kd = pidCoefficients.d;
            if(targetPosition != past_target){
                integral = 0;
                lastError = 0;
            }
            past_target = targetPosition;
            if(last_power != 0 && gamepad1.right_stick_y == 0){
                targetPosition = getSteeringPosition();
            }
            last_power = gamepad1.right_stick_y;
            int currentPosition = getSteeringPosition();  // Get the current steering position (using an encoder or similar)
            double error = targetPosition - currentPosition;  // Calculate error

            // Proportional, Integral, Derivative terms
            integral += error * runtime.seconds();
            double derivative = (error - lastError) / runtime.seconds();

            // PID output
            double pidOutput = kp * error + ki * integral + kd * derivative;

            // Apply output to the steering servo (make sure to scale appropriately for the servo input range)
            double steeringPower = Range.clip(pidOutput, -1.0, 1.0);

            steeringServo.setPower(gamepad1.right_stick_y == 0 ? steeringPower : gamepad1.right_stick_y);


            // Drive motor example (you can add more logic here based on your drive control)
            driveMotor.setPower(gamepad1.left_stick_y);  // Set some constant power for the drive motor

            // Update last error and reset the timer
            lastError = error;
            runtime.reset();

            // Telemetry for debugging
            telemetry.addData("Target Position", targetPosition);
            telemetry.addData("Current Position", currentPosition);
            telemetry.addData("Error", error);
            telemetry.addData("Steering Power", steeringPower);
            telemetry.update();
        }
    }

    // Placeholder method to get the curre
    // nt position of the steering servo
    private int getSteeringPosition() {
        // You will need to implement this based on your setup (e.g., using an encoder or potentiometer)
        // For example, if you are using an encoder, you could convert encoder counts to a 0-1 range.
        return driveMotor.getCurrentPosition();
    }
}
