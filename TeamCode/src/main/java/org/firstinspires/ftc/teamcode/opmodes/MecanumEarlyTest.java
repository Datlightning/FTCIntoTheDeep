package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.MecaTank;


@TeleOp
@Config
public class MecanumEarlyTest extends LinearOpMode {
    Intake intake;
    MecaTank mecaTank;
    boolean x_pressed = false;
    boolean a_pressed = false;
    boolean bumper_pressed = false;
    public void runOpMode(){
        mecaTank = new MecaTank(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);
        intake.init();
        mecaTank.init();
        waitForStart();
        while(!isStopRequested() && opModeIsActive()){
            mecaTank.setPowers(gamepad1.left_stick_y, gamepad1.right_stick_y, gamepad1.left_trigger, gamepad1.right_trigger);
            if(gamepad1.x && !x_pressed){
                x_pressed = true;
                if (intake.isClawOpen()) {
                    intake.closeClaw();
                }else{
                    intake.openClaw();
                }
            }else if(!gamepad1.x){
                x_pressed = false;
            }

            if(gamepad1.a && !a_pressed){
                a_pressed = true;
                if (intake.isWristOpen()) {
                    intake.foldWrist();
                }else{
                    intake.extendWrist();
                }
            }else if(!gamepad1.a){
                a_pressed = false;
            }

            if(gamepad1.left_bumper && !bumper_pressed){
                bumper_pressed = true;
                intake.setWristPWM(!intake.WristPWMOn());
            }else if(!gamepad1.left_bumper){
                bumper_pressed = false;
            }

            mecaTank.telemetry();
            intake.telemetry();

            intake.update();
            mecaTank.update();
            telemetry.update();
        }
    }
}
