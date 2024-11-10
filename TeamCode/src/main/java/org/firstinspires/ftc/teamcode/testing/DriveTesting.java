package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.MecaTank;


@TeleOp
@Disabled
public class DriveTesting extends LinearOpMode {
    MecaTank mecaTank;

    public void runOpMode(){
        mecaTank = new MecaTank(hardwareMap, telemetry);

        mecaTank.init();
        waitForStart();
        while(!isStopRequested() && opModeIsActive()){
            mecaTank.setPowers(gamepad1.left_stick_y, gamepad1.right_stick_y, gamepad1.left_trigger, gamepad1.right_trigger);



            mecaTank.telemetry();
            mecaTank.update();
            telemetry.update();
        }
    }
}
