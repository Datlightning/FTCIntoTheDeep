package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.teamcode.subsystems.Intake;

@Config
public class RobotConstants {
    //horizontal slides1
    public static double claw_fully_open = 0.8, claw_closed = 1, claw_open=0.9, claw_flat = 0.57;
    public static double wrist_folded = 0.9, wrist_extended=0, floor_pickup_position = 0.56, specimen_deliver = 0.6;//update the 0 and 90 degrees as well.
    public static double TOO_FAR = 2.5, TOO_CLOSE = 1.3, GIVE_UP = 6, TARGET = 1.75;
    public static Intake intake;
    public static String intakeMotor = "intake_motor";//ehub 0
    public static String slidesMotor = "slide_motor";//ehub 1
    public static String claw_servo = "claw_servo";//ehub 4
    public static String wrist_servo = "wrist_servo";//ehub 3

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
