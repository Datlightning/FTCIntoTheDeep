package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.teamcode.subsystems.Intake;

@Config
public class RobotConstants {
    //horizontal slides1
    public static double claw_fully_open = 0.8, claw_closed = 1, claw_open=0.9, claw_flat = 0.57;
    public static double wrist_folded = 0, wrist_extended=30, floor_pickup_position = 102, specimen_deliver = 60;//update the 0 and 90 degrees as well.
    public static double TOO_FAR = 3.8, TOO_CLOSE = 2, GIVE_UP = 6, TARGET = 2.9;
    public static Intake intake;
    public static int ARM_LIMIT = 1600;
    public static String intakeMotor = "intake_motor";//ehub 0
    public static String slidesMotor = "slide_motor";//ehub 1
    public static String claw_servo = "claw_servo";//ehub 4 (yellow servo connector)
    public static String left_servo = "left_servo";//ehub 3 (black tape)
    public static String right_servo = "right_servo";//ehub 1 (copious lack of black tape)


    public static String green_led = "front_led_green";
    public static String red_led = "front_led_red";
    public static String fr = "frontRight";//chub 0 && perp deadwheel
    public static String br = "backRight";//chub 1
    public static String fl = "frontLeft";//chub 2
    public static String bl = "backLeft";//chub 3 && par deadwheel

    public static String magnet_sensor = "magnet_sensor";//digital 0
    public static String distance = "sensor_distance";//i2c 1

    public static String rear_distance = "rear_distance";//ehub i2c 1

    public static LazyImu imu;
    public static boolean imu_init = false;
    public static boolean auto_transfer = false;
    public static int offset = 0;
}
