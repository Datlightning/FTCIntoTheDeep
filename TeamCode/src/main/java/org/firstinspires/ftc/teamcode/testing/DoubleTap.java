package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp(name = "Double Tap Detection", group = "TeleOp")
public class DoubleTap extends LinearOpMode {

    // Variables to track tap state
    boolean previousButtonState  = false;
    boolean currentButtonState = false;
    boolean waitingForSecondTapA = false;
    double lastTapTime = 0;

    int taps = 0;
    double tapDelayThreshold = 0.3; // Time window to detect double tap (seconds)
    double tapTimeOut = 0.35; // Max time after first tap to confirm single tap

    double currentTaps = 0;

    @Override
    public void runOpMode() {
        // Wait for the start button
        telemetry.addData("Status", "Waiting for Start");
        telemetry.update();
        waitForStart();

        // Start the opmode loop
        while (opModeIsActive()) {
            // Update previous and current button states
            previousButtonState = currentButtonState;
            currentButtonState = gamepad1.a; // Check the A button state (you can change this to any other button)
            if(getRuntime() - lastTapTime > tapTimeOut) {
                currentTaps = taps;
                telemetry.addLine("Taps " + taps);
                lastTapTime = getRuntime();
                taps = 0;
            }

            if(currentButtonState && !previousButtonState){
                taps += 1;
                lastTapTime = getRuntime();
            }
            // Check taps based on the previous and current state of the button
            if(currentTaps > 0){
                if(currentTaps == 1){
                    telemetry.addLine("progress forward");
                }else if(currentTaps == 2){
                    telemetry.addLine("run.");
                }
                currentTaps = 0;
            }
            telemetry.update();
        }
    }

}