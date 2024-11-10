package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Config
@TeleOp
@Disabled
public class ServoExTesting extends LinearOpMode {
    ServoImplEx servo;
    @Override
    public void runOpMode() throws InterruptedException {
        servo = hardwareMap.get(ServoImplEx.class, "wrist_servo");
    }
}
