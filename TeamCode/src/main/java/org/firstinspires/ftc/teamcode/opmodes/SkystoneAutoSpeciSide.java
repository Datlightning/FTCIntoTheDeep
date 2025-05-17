package org.firstinspires.ftc.teamcode.opmodes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
public class SkystoneAutoSpeciSide extends LinearOpMode{
    public DcMotor RightFront;
    public DcMotor LeftFront;
    public DcMotor RightBack;
    public DcMotor LeftBack;

    @Override
    public void runOpMode() throws InterruptedException{
        RightFront = hardwareMap.get(DcMotor.class, "RFM");
        LeftFront = hardwareMap.get(DcMotor.class, "LFM");
        RightBack = hardwareMap.get(DcMotor.class, "RBM");
        LeftBack = hardwareMap.get(DcMotor.class, "LBM");

        waitForStart();

        while(!isStopRequested()){
            if(gamepad1.a){
                //moveForward();
                RightFront.setPower(-0.5);
                RightBack.setPower(-0.5);
                LeftFront.setPower(0.5);
                LeftBack.setPower(0.5);
            }
        }
    }
    private void moveForward(double speed, int time){
        RightFront.setPower(speed);
        LeftFront.setPower(speed);
        RightBack.setPower(speed);
        LeftBack.setPower(speed);
        sleep(time);
    }
    private void moveBack(double speed, int time){
        RightFront.setPower(-speed);
        LeftFront.setPower(-speed);
        RightBack.setPower(-speed);
        LeftBack.setPower(-speed);
        sleep(time);
    }

}
