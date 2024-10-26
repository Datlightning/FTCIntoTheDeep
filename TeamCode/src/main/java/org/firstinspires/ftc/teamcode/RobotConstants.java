package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.subsystems.Intake;

@Config
public class RobotConstants {
    //horizontal slides1
    public static double claw_fully_open = 0.8, claw_closed = 0.95, claw_open=0.89;
    public static double wrist_folded = 0.9, wrist_extended=0, floor_pickup_position = 0.48;
    public static Intake intake;
    public static String intakeMotor = "intake_motor";//ehub 0
    public static String slidesMotor = "slide_motor";//ehub 1
    public static String claw_servo = "claw_servo";//chub 0
    public static String wrist_servo = "wrist_servo";//chub 1

    public static String fr = "frontRight";//chub 0 && perp deadwheel
    public static String br = "backRight";//chub 1
    public static String fl = "frontLeft";//chub 2
    public static String bl = "backLeft";//chub 3 && par deadwheel

    public static String magnet_sensor = "magnet_sensor";

    public static boolean auto_transfer = false;
    public static int offset = 0;
}
