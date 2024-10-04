package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RobotConstants {
    //horizontal slides1
    public static double slide_in = 0.5, slide_out = 0.36;
    public static double claw_fully_open = 0, claw_closed = 0.46, claw_open=0.3;
    public static double wrist_folded = 0, wrist_extended=0.4;

    public static String intakeMotor = "intake_motor";//ehub 0
    public static String slidesMotor = "slide_motor";
    public static String claw_servo = "claw_servo";//chub 0
    public static String wrist_servo = "wrist_servo";//chub 1

    public static String fr = "frontRight";//chub 0
    public static String br = "backRight";//chub 1
    public static String fl = "frontLeft";//chub 2
    public static String bl = "backLeft";//chub 3
}
