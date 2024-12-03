package org.firstinspires.ftc.teamcode.library;

import java.util.HashMap;
import java.util.Map;

public class MultiClick {
    private static final double TAP_THRESHOLD = 0.3;

    // Each button will store taps and last tap time.
    private static class ButtonState {
        int taps;
        double lastTapTime;
        int currentTaps;

        public ButtonState() {
            this.taps = 0;
            this.lastTapTime = 0;
            this.currentTaps = 0;
        }
    }

    private final Map<String, ButtonState> buttonStates = new HashMap<>();

    // Function to reset taps after threshold time has passed
    public void update(String buttonId, double currentTime, boolean buttonPressed) {
        ButtonState state = buttonStates.computeIfAbsent(buttonId, k -> new ButtonState());

        // Check if more than TAP_THRESHOLD seconds have passed since the last tap
        if (currentTime - state.lastTapTime > TAP_THRESHOLD) {
            state.currentTaps = state.taps;
            state.lastTapTime = currentTime;
            state.taps = 0;
        }

        // Detect a button press (i.e., current press is true, previous press is false)
        if (buttonPressed) {
            state.taps += 1;
            state.lastTapTime = currentTime;
        }
    }

    // Get the current taps for a button
    public int getTaps(String buttonId) {
        return buttonStates.getOrDefault(buttonId, new ButtonState()).currentTaps;
    }

    public void clearTaps(String buttonId){
        buttonStates.getOrDefault(buttonId, new ButtonState()).currentTaps = 0;

    }
}
