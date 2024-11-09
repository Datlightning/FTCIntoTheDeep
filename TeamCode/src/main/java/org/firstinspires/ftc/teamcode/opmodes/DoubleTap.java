package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp(name = "Double Tap Detection", group = "TeleOp")
public class DoubleTap extends LinearOpMode {

    // Variables to track tap state
    boolean previousButtonState = false;
    boolean currentButtonState = false;
    boolean waitingForSecondTap = false;
    double firstTapTime = 0;
    public static double doubleTapThreshold = 0.8; // 300ms max interval for a double tap
    boolean doubleTapped = false;
    boolean singleClicked = false;
    public static double clickTimeout = 0.85; // Timeout to register as a single click

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

            // Check taps based on the previous and current state of the button
            checkTap();

            // Log data to telemetry
            telemetry.addData("Previous Button State", previousButtonState);
            telemetry.addData("Current Button State", currentButtonState);
            telemetry.addData("Waiting for Second Tap", waitingForSecondTap);
            telemetry.addData("Double Tapped", doubleTapped);
            telemetry.addData("Single Clicked", singleClicked);
            telemetry.update();
        }
    }

    public void checkTap() {
        if (waitingForSecondTap && (getRuntime() - firstTapTime) > clickTimeout) {
            singleClicked = true;
            doSomethingOnSingleClick();
            waitingForSecondTap = false; // Reset waiting state after single click
        }
        // Detect button press (when transitioning from not pressed to pressed)
        if (currentButtonState && !previousButtonState) {
            // First tap detected
            if (!waitingForSecondTap) {
                firstTapTime = getRuntime();
                waitingForSecondTap = true;
            } else {
                // If waiting for second tap and within threshold, register double tap
                if ((getRuntime() - firstTapTime) <= doubleTapThreshold) {
                    doubleTapped = true;
                    waitingForSecondTap = false; // Reset after detecting double tap
                }
            }
        }

        // Detect button release (use this to trigger single clicks)


        // If double-tapped, log the event
        if (doubleTapped) {
            doSomethingOnDoubleTap();
            doubleTapped = false; // Reset double tap state after handling
        }

        // Reset single click state after handling
        singleClicked = false;
    }

    public void doSomethingOnDoubleTap() {
        // Define the action you want to trigger on double tap
        telemetry.log().add("Double tap detected!");
    }

    public void doSomethingOnSingleClick() {
        // Define the action you want to trigger on single click
        telemetry.log().add("Single click detected!");
    }
}