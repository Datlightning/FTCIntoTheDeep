package org.firstinspires.ftc.teamcode.library;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Consumer;

public class BiDirectionalStateMachine {
    private Map<String, List<Runnable>> states = new HashMap<>();
    private Map<String, String> transitionsForward = new HashMap<>();
    private Map<String, String> transitionsBackward = new HashMap<>();
    private String currentState;

    // Initialize the state machine with states and their actions
    public BiDirectionalStateMachine(String initialState) {
        this.currentState = initialState;
    }

    // Add a state with a list of actions
    public void addState(String state, List<Runnable> actions) {
        states.put(state, actions);
    }

    // Define transitions between states
    public void addTransition(String fromState, String toState) {
        transitionsForward.put(fromState, toState);
        transitionsBackward.put(toState, fromState);
    }

    // Run the current state's actions
    public void runCurrentState() {
        List<Runnable> actions = states.get(currentState);
        if (actions != null) {
            actions.forEach(Runnable::run);
        } else {
            System.out.println("No actions for state: " + currentState);
        }
    }

    // Move to the next state
    public void moveToNextState() {
        String nextState = transitionsForward.get(currentState);
        if (nextState != null) {
            currentState = nextState;
        }
    }

    // Move to the previous state
    public void moveToPreviousState() {
        String previousState = transitionsBackward.get(currentState);
        if (previousState != null) {
            currentState = previousState;
        }
    }
}
