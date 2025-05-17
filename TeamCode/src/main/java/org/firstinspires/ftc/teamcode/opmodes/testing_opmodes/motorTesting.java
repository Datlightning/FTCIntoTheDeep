package org.firstinspires.ftc.teamcode.opmodes.testing_opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.PwmControl;

@Config
@TeleOp
public class motorTesting extends LinearOpMode{
    private DcMotor FrontLeft;
    private DcMotor FrontRight;
    private DcMotor BackLeft;
    private DcMotor BackRight;
    private DcMotor Arm;

    @Override
    public void runOpMode() throws InterruptedException{
        if(gamepad1.a){
            FrontLeft.setPower(0.4);
        }else{
            FrontLeft.setPower(0);
        }
        if(gamepad1.b){
            FrontRight.setPower(0.4);
        }else{
            FrontRight.setPower(0);
        }
        if(gamepad1.y){
            BackLeft.setPower(0.4);
        }else{
            BackLeft.setPower(0);
        }
        if(gamepad1.x){
            BackRight.setPower(0.4);
        }else{
            BackRight.setPower(0);
        }
        telemetry.addData("Motor Power(FL): ", FrontLeft.getPower());
        telemetry.addData("Motor Power(FR): ", FrontRight.getPower());
        telemetry.addData("Motor Power(BL): ", BackLeft.getPower());
        telemetry.addData("Motor Power(BR)", BackRight.getPower());
    }
}
