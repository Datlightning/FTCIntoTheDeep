package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;


public class ServoExTesting extends TestingOpMode {
    ServoImplEx servo;
    @Override
    public void runOpMode() throws InterruptedException {
        makeTelemetry();
        servo = hardwareMap.get(ServoImplEx.class, "wrist_servo");
    }
}
