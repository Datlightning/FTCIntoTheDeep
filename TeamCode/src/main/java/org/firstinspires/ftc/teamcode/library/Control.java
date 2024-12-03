package org.firstinspires.ftc.teamcode.library;

public class Control {
    public static double motionProfile(double maxAcceleration, double maxVelocity, double distance, double elapsedTime) {
        // Track whether the distance is positive or negative
        int direction = distance < 0 ? -1 : 1;
        distance = Math.abs(distance);  // Work with absolute distance

        // Calculate the time it takes to accelerate to max velocity
        double accelerationDt = maxVelocity / maxAcceleration;

        // If we can't accelerate to max velocity in the given distance, adjust accordingly
        double halfwayDistance = distance / 2.0;
        double accelerationDistance = 0.5 * maxAcceleration * Math.pow(accelerationDt, 2);

        if (accelerationDistance > halfwayDistance) {
            accelerationDt = Math.sqrt(halfwayDistance / (0.5 * maxAcceleration));
        }

        accelerationDistance = 0.5 * maxAcceleration * Math.pow(accelerationDt, 2);

        // Recalculate max velocity based on the adjusted acceleration time
        maxVelocity = maxAcceleration * accelerationDt;

        // Deceleration happens at the same rate as acceleration
        double decelerationDt = accelerationDt;

        // Calculate the distance covered during the cruise (at max velocity)
        double cruiseDistance = distance - 2 * accelerationDistance;
        double cruiseDt = cruiseDistance / maxVelocity;
        double decelerationTime = accelerationDt + cruiseDt;

        // Total time for the motion profile (acceleration + cruise + deceleration)
        double entireDt = accelerationDt + cruiseDt + decelerationDt;

        // If elapsed time exceeds the total time of the profile, return the full distance
        if (elapsedTime > entireDt) {
            return direction * distance;
        }

        // If we're in the acceleration phase
        if (elapsedTime < accelerationDt) {
            // Use the kinematic equation for acceleration: s = 0.5 * a * t^2
            double position = 0.5 * maxAcceleration * Math.pow(elapsedTime, 2);
            return direction * (int) Math.round(position);
        }

        // If we're in the cruising phase (constant velocity)
        else if (elapsedTime < decelerationTime) {
            accelerationDistance = 0.5 * maxAcceleration * Math.pow(accelerationDt, 2);
            double cruiseCurrentDt = elapsedTime - accelerationDt;

            // Use the kinematic equation for constant velocity: s = v * t
            double position = accelerationDistance + maxVelocity * cruiseCurrentDt;
            return direction * (int) Math.round(position);
        }

        // If we're in the deceleration phase
        else {
            accelerationDistance = 0.5 * maxAcceleration * Math.pow(accelerationDt, 2);
            cruiseDistance = maxVelocity * cruiseDt;
            double decelerationElapsedTime = elapsedTime - decelerationTime;

            // Use the kinematic equations for deceleration: s = v * t - 0.5 * a * t^2
            double position = accelerationDistance + cruiseDistance
                    + maxVelocity * decelerationElapsedTime
                    - 0.5 * maxAcceleration * Math.pow(decelerationElapsedTime, 2);
            return direction * (int) Math.round(position);
        }
    }

    public static double motionProfileVelo(double V_MAX, double ACCEL_MAX, double DECEL_MAX, double DISTANCE, double time) {
        int DIRECTION = DISTANCE < 0 ? -1 : 1;
        DISTANCE = Math.abs(DISTANCE);

        double ACCEL_TIME = V_MAX / ACCEL_MAX;
        double DECEL_TIME = V_MAX / DECEL_MAX;

        double MIN_X = 0.5 * ACCEL_MAX * Math.pow(ACCEL_TIME, 2) + 0.5 * DECEL_MAX * Math.pow(DECEL_TIME, 2);
        double CRUISE_TIME = 0;

        if (MIN_X > DISTANCE) {
            ACCEL_TIME = Math.sqrt(2 * DISTANCE * DECEL_MAX / (ACCEL_MAX * (ACCEL_MAX + DECEL_MAX)));
            DECEL_TIME = (ACCEL_MAX / DECEL_MAX) * ACCEL_TIME;
            V_MAX = ACCEL_TIME * ACCEL_MAX;
        } else {
            CRUISE_TIME = (DISTANCE - MIN_X) / V_MAX;
        }

        if (time > ACCEL_TIME + CRUISE_TIME + DECEL_TIME) {
            return 0;
        }

        if (time < ACCEL_TIME) {
            return ACCEL_MAX * time * DIRECTION;
        } else if (time < CRUISE_TIME + ACCEL_TIME) {
            return DIRECTION * V_MAX;
        } else {
            double CURRENT_DECEL_TIME = time - ACCEL_TIME - CRUISE_TIME;
            return DIRECTION * (V_MAX - (CURRENT_DECEL_TIME * DECEL_MAX));
        }
    }
    public static double motionProfile(double V_MAX, double ACCEL_MAX, double DECEL_MAX, double DISTANCE, double time) {
        double ACCEL_TIME = V_MAX / ACCEL_MAX;
        double DECEL_TIME = V_MAX / DECEL_MAX;
        int DIRECTION = DISTANCE < 0 ? -1 : 1;
        DISTANCE = Math.abs(DISTANCE);

        double MIN_X = 0.5 * ACCEL_MAX * Math.pow(ACCEL_TIME, 2) + 0.5 * DECEL_MAX * Math.pow(DECEL_TIME, 2);
        double CRUISE_TIME = 0;

        if (MIN_X > DISTANCE) {
            ACCEL_TIME = Math.sqrt(2 * DISTANCE * DECEL_MAX / (ACCEL_MAX * ACCEL_MAX + ACCEL_MAX * DECEL_MAX));
            DECEL_TIME = (ACCEL_MAX / DECEL_MAX) * ACCEL_TIME;
            V_MAX = ACCEL_TIME * ACCEL_MAX;
        } else {
            CRUISE_TIME = (DISTANCE - MIN_X) / V_MAX;
        }

        if (time > ACCEL_TIME + CRUISE_TIME + DECEL_TIME) {
            return DIRECTION * DISTANCE;
        }

        if (time < ACCEL_TIME) {
            return DIRECTION * 0.5 * ACCEL_MAX * Math.pow(time, 2);
        } else if (time < CRUISE_TIME + ACCEL_TIME) {
            return DIRECTION * (0.5 * ACCEL_MAX * Math.pow(ACCEL_TIME, 2) + (time - ACCEL_TIME) * V_MAX);
        } else {
            double CURRENT_DECEL_TIME = time - ACCEL_TIME - CRUISE_TIME;
            return DIRECTION * (
                    0.5 * ACCEL_MAX * Math.pow(ACCEL_TIME, 2) +
                            CRUISE_TIME * V_MAX +
                            V_MAX * CURRENT_DECEL_TIME -
                            0.5 * DECEL_MAX * Math.pow(CURRENT_DECEL_TIME, 2)
            );
        }
    }
    public static double motionProfileTime(double V_MAX, double ACCEL_MAX, double DECEL_MAX, double DISTANCE) {
        double ACCEL_TIME = V_MAX / ACCEL_MAX;
        double DECEL_TIME = V_MAX / DECEL_MAX;
        int DIRECTION = DISTANCE < 0 ? -1 : 1;
        DISTANCE = Math.abs(DISTANCE);

        double MIN_X = 0.5 * ACCEL_MAX * Math.pow(ACCEL_TIME, 2) + 0.5 * DECEL_MAX * Math.pow(DECEL_TIME, 2);
        double CRUISE_TIME = 0;

        if (MIN_X > DISTANCE) {
            ACCEL_TIME = Math.sqrt(2 * DISTANCE * DECEL_MAX / (ACCEL_MAX * ACCEL_MAX + ACCEL_MAX * DECEL_MAX));
            DECEL_TIME = (ACCEL_MAX / DECEL_MAX) * ACCEL_TIME;
            V_MAX = ACCEL_TIME * ACCEL_MAX;
        } else {
            CRUISE_TIME = (DISTANCE - MIN_X) / V_MAX;
        }

        return ACCEL_TIME + CRUISE_TIME + DECEL_TIME;

    }
    public static int motionProfile(double maxAcceleration, double maxVelocity, int distance, double elapsedTime) {
        // Track whether the distance is positive or negative
        int direction = distance < 0 ? -1 : 1;
        distance = Math.abs(distance);  // Work with absolute distance

        // Calculate the time it takes to accelerate to max velocity
        double accelerationDt = maxVelocity / maxAcceleration;

        // If we can't accelerate to max velocity in the given distance, adjust accordingly
        double halfwayDistance = distance / 2.0;
        double accelerationDistance = 0.5 * maxAcceleration * Math.pow(accelerationDt, 2);

        if (accelerationDistance > halfwayDistance) {
            accelerationDt = Math.sqrt(halfwayDistance / (0.5 * maxAcceleration));
        }

        accelerationDistance = 0.5 * maxAcceleration * Math.pow(accelerationDt, 2);

        // Recalculate max velocity based on the adjusted acceleration time
        maxVelocity = maxAcceleration * accelerationDt;

        // Deceleration happens at the same rate as acceleration
        double decelerationDt = accelerationDt;

        // Calculate the distance covered during the cruise (at max velocity)
        double cruiseDistance = distance - 2 * accelerationDistance;
        double cruiseDt = cruiseDistance / maxVelocity;
        double decelerationTime = accelerationDt + cruiseDt;

        // Total time for the motion profile (acceleration + cruise + deceleration)
        double entireDt = accelerationDt + cruiseDt + decelerationDt;

        // If elapsed time exceeds the total time of the profile, return the full distance
        if (elapsedTime > entireDt) {
            return direction * distance;
        }

        // If we're in the acceleration phase
        if (elapsedTime < accelerationDt) {
            // Use the kinematic equation for acceleration: s = 0.5 * a * t^2
            double position = 0.5 * maxAcceleration * Math.pow(elapsedTime, 2);
            return direction * (int) Math.round(position);
        }

        // If we're in the cruising phase (constant velocity)
        else if (elapsedTime < decelerationTime) {
            accelerationDistance = 0.5 * maxAcceleration * Math.pow(accelerationDt, 2);
            double cruiseCurrentDt = elapsedTime - accelerationDt;

            // Use the kinematic equation for constant velocity: s = v * t
            double position = accelerationDistance + maxVelocity * cruiseCurrentDt;
            return direction * (int) Math.round(position);
        }

        // If we're in the deceleration phase
        else {
            accelerationDistance = 0.5 * maxAcceleration * Math.pow(accelerationDt, 2);
            cruiseDistance = maxVelocity * cruiseDt;
            double decelerationElapsedTime = elapsedTime - decelerationTime;

            // Use the kinematic equations for deceleration: s = v * t - 0.5 * a * t^2
            double position = accelerationDistance + cruiseDistance
                    + maxVelocity * decelerationElapsedTime
                    - 0.5 * maxAcceleration * Math.pow(decelerationElapsedTime, 2);
            return direction * (int) Math.round(position);
        }
    }
    public static double motionProfileTime(double maxAcceleration, double maxVelocity, int distance) {
        // Track whether the distance is positive or negative
        int direction = distance < 0 ? -1 : 1;
        distance = Math.abs(distance);  // Work with absolute distance

        // Calculate the time it takes to accelerate to max velocity
        double accelerationDt = maxVelocity / maxAcceleration;

        // If we can't accelerate to max velocity in the given distance, adjust accordingly
        double halfwayDistance = distance / 2.0;
        double accelerationDistance = 0.5 * maxAcceleration * Math.pow(accelerationDt, 2);

        if (accelerationDistance > halfwayDistance) {
            accelerationDt = Math.sqrt(halfwayDistance / (0.5 * maxAcceleration));
        }

        accelerationDistance = 0.5 * maxAcceleration * Math.pow(accelerationDt, 2);

        // Recalculate max velocity based on the adjusted acceleration time
        maxVelocity = maxAcceleration * accelerationDt;

        // Deceleration happens at the same rate as acceleration
        double decelerationDt = accelerationDt;

        // Calculate the distance covered during the cruise (at max velocity)
        double cruiseDistance = distance - 2 * accelerationDistance;
        double cruiseDt = cruiseDistance / maxVelocity;
        double decelerationTime = accelerationDt + cruiseDt;

        // Total time for the motion profile (acceleration + cruise + deceleration)
        return accelerationDt + cruiseDt + decelerationDt;
    }


    public static double motionProfileVelo(double maxAcceleration, double maxVelocity, int distance, double elapsedTime){
// Track whether the distance is positive or negative
        int direction = distance < 0 ? -1 : 1;
        distance = Math.abs(distance);  // Work with absolute distance

// Calculate the time it takes to accelerate to max velocity
        double accelerationDt = maxVelocity / maxAcceleration;

// If we can't accelerate to max velocity in the given distance, adjust accordingly
        double halfwayDistance = distance / 2.0;
        double accelerationDistance = 0.5 * maxAcceleration * Math.pow(accelerationDt, 2);

        if (accelerationDistance > halfwayDistance) {
            accelerationDt = Math.sqrt(halfwayDistance / (0.5 * maxAcceleration));
        }

        accelerationDistance = 0.5 * maxAcceleration * Math.pow(accelerationDt, 2);

// Deceleration happens at the same rate as acceleration
        double decelerationDt = accelerationDt;

// Calculate the distance covered during the cruise (at max velocity)
        double cruiseDistance = distance - 2 * accelerationDistance;
        double cruiseDt = cruiseDistance / maxVelocity;
        double decelerationTime = accelerationDt + cruiseDt;

// Total time for the motion profile (acceleration + cruise + deceleration)
        double entireDt = accelerationDt + cruiseDt + decelerationDt;

// If elapsed time exceeds the total time of the profile, return zero velocity (since the motion is complete)
        if (elapsedTime > entireDt) {
            return 0;  // Velocity is 0 when the motion is complete
        }

// If we're in the acceleration phase
        if (elapsedTime < accelerationDt) {
            // Velocity during acceleration phase: v = a * t
            double velocity = maxAcceleration * elapsedTime;
            return direction * velocity;
        }

// If we're in the cruising phase (constant velocity)
        else if (elapsedTime < decelerationTime) {
            return direction * maxVelocity;  // Constant max velocity during cruise
        }

// If we're in the deceleration phase
        else {
            double decelerationElapsedTime = elapsedTime - decelerationTime;

            // Velocity during deceleration phase: v = maxVelocity - a * t
            double velocity = maxVelocity - maxAcceleration * decelerationElapsedTime;
            return direction * velocity;
        }

    }


    public static double lowPassFilter(double current_value, double past_value, double filterValue){ return current_value * filterValue + (1 - filterValue) * past_value; }

}
