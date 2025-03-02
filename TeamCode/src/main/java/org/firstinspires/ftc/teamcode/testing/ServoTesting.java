package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Intake;


public class ServoTesting extends TestingOpMode {

    Intake intake;
    ElapsedTime timer;
    double current_time = 0;
    double position = 0;
    double increment = 0.05;
    @Override
    public void runOpMode() throws InterruptedException {
        makeTelemetry();
        timer = new ElapsedTime();
        intake = new Intake(hardwareMap, telemetry);

        waitForStart();
        while(!isStopRequested() && opModeIsActive()){
            if(timer.time() - current_time > 0.3){
                position += increment;
                current_time = timer.time();
            }
            if(position >= 1){
                increment = -0.05;
            }else if(position <= 0){
                increment= 0.05;
            }
            telemetry.addData("Position", position);
            intake.moveWrist(position);
            telemetry.update();
        }

    }
}
