package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp
public class BTeleOp extends LinearOpMode {
    private DcMotor FrontLeft;
    private DcMotor FrontRight;
    private DcMotor BackLeft;
    private DcMotor BackRight;

    private DcMotor Arm;

    private Servo claw;

    private DcMotor rigging;

    @Override
    public void runOpMode() throws InterruptedException {
        FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        BackRight = hardwareMap.get(DcMotor.class, "BackRight");
        Arm = hardwareMap.get(DcMotor.class, "Arm");
        claw = hardwareMap.get(Servo.class, "claw");
        rigging=hardwareMap.get(DcMotor.class, "rigging");
        waitForStart();
        while (!isStopRequested() && opModeIsActive()) {
            FrontLeft.setPower(gamepad1.left_stick_y);
            BackLeft.setPower(gamepad1.left_stick_y);
            FrontRight.setPower(-gamepad1.right_stick_y);
            BackRight.setPower(-gamepad1.right_stick_y);

            if(gamepad1.left_bumper){
                rigging.setPower(-0.4);
            }else{
                rigging.setPower(0);
            }
            if(gamepad1.right_bumper){
                rigging.setPower(0.4);
            }else{
                rigging.setPower(0);
            }
            if(gamepad1.y){
                rigging.setPower(0.85);
                FrontLeft.setPower(-0.45);
                FrontRight.setPower(-0.45);
            }
        }
    }
}